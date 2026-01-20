use defmt::*;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::Output,
    mode::Async,
    spi::{Spi, mode::Master},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

type SpiError = embassy_stm32::spi::Error;

#[macro_export]
macro_rules! encode_reg8 {
    (base: $base:expr, { $($val:expr => $shift:literal, $width:literal),* $(,)? }) => {
        {
            let mut v: u8 = $base;
            $(
                let mask: u8 = ((1u16 << $width) - 1) as u8;
                let shifted_mask: u8 = mask << $shift;
                v = (v & !shifted_mask) | (($val as u8 & mask) << $shift);
            )*
            v
        }
    };

    ({ $($val:expr => $shift:literal, $width:literal),* $(,)? }) => {
        $crate::encode_reg8!(base: 0, { $($val => $shift, $width),* })
    };
}

#[derive(defmt::Format)]
pub enum ErrorAdc {
    WrongConfig,
    Spi(SpiError),
}

#[repr(u8)]
pub enum SensorMode {
    ADC = 0,
    Temp = 1,
}

#[repr(u8)]
pub enum OperationMode {
    SingleShot = 1,
    Continuous(u16) = 0,
}

#[repr(u8)]
pub enum FSR {
    FSR6_144V = 0b000,
    FSR4_096V = 0b001,
    FSR2_048V = 0b010,
    FSR1_024V = 0b011,
    FSR0_512V = 0b100,
    FSR0_256V = 0b101,
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum DataRate {
    SPS8 = 0b000,
    SPS16 = 0b001,
    SPS32 = 0b010,
    SPS64 = 0b011,
    SPS128 = 0b100,
    SPS250 = 0b101,
    SPS475 = 0b110,
    SPS860 = 0b111,
}

#[repr(u8)]
pub enum Channel {
    CH1 = 0b100,
    CH2 = 0b101,
    CH3 = 0b110,
}

impl DataRate {
    pub fn get_conversion_time(&self) -> u64 {
        let base_us = match self {
            DataRate::SPS8 => 125_000,
            DataRate::SPS16 => 62_500,
            DataRate::SPS32 => 31_250,
            DataRate::SPS64 => 15_625,
            DataRate::SPS128 => 7_813,
            DataRate::SPS250 => 4_000,
            DataRate::SPS475 => 2_105,
            DataRate::SPS860 => 1_163,
        };

        (base_us * 115) / 100
    }
}
pub struct Adc<'d> {
    spi: &'d Mutex<ThreadModeRawMutex, Spi<'d, Async, Master>>,
    cs: Output<'d>,
    int: ExtiInput<'d>,
    temp_drift_correction: bool,
    offset_error: Option<[i16; 3]>,
    gain_error: Option<[i16; 3]>,
    pub pull_up_enable: bool,
    data_rate: DataRate,
    conversion_time: u64,
}

impl<'d> Adc<'d> {
    pub fn new(
        spi: &'d Mutex<ThreadModeRawMutex, Spi<'d, Async, Master>>,
        cs: Output<'d>,
        int: ExtiInput<'d>,
        temp_drift_correction: bool,
    ) -> Self {
        let dr = DataRate::SPS128;
        Self {
            spi,
            cs,
            int,
            temp_drift_correction,
            offset_error: None,
            gain_error: None,
            pull_up_enable: false,
            data_rate: dr,
            conversion_time: dr.get_conversion_time(),
        }
    }
    pub fn set_offset_error(&mut self, ch1: i16, ch2: i16, ch3: i16) {
        self.offset_error = Some([ch1, ch2, ch3]);
    }

    pub fn set_gain_error(&mut self, ch1: i16, ch2: i16, ch3: i16) {
        self.gain_error = Some([ch1, ch2, ch3]);
    }

    pub fn set_data_rate(&mut self, rate: DataRate) {
        self.data_rate = rate;
        self.conversion_time = rate.get_conversion_time();
    }

    pub fn activate_temp_drift_correction(&mut self) -> Result<(), ErrorAdc> {
        if self.gain_error.is_some() && self.offset_error.is_some() {
            self.temp_drift_correction = true;
            Ok(())
        } else {
            error!("Gain Error and Offset Error need to be set");
            Err(ErrorAdc::WrongConfig)
        }
    }

    async fn write_register(&mut self, config_msb: u8, config_lsb: u8) -> Result<(), ErrorAdc> {
        let mut buf = [config_msb, config_lsb];
        self.cs.set_low();
        let mut spi = self.spi.lock().await;
        let spi: &mut Spi<'_, Async, Master> = &mut spi;
        spi.transfer_in_place(&mut buf)
            .await
            .map_err(|e| ErrorAdc::Spi(e))?;
        self.cs.set_high();
        Ok(())
    }

    async fn read_register(&mut self) -> Result<[u8; 2], ErrorAdc> {
        let mut buf = [0u8; 2];
        self.cs.set_low();
        let mut spi = self.spi.lock().await;
        let spi: &mut Spi<'_, Async, Master> = &mut spi;
        spi.read(&mut buf).await.map_err(|e| ErrorAdc::Spi(e))?;
        self.cs.set_high();
        Ok(buf)
    }

    async fn read_data_adc(
        &mut self,
        channel: Channel,
        fsr: FSR,
        data_rate: DataRate,
        mode: SensorMode,
        operating: OperationMode,
    ) -> Result<i16, ErrorAdc> {
        match operating {
            OperationMode::Continuous(sample_count) => {
                let config_msb = encode_reg8!({
                    0 => 15, 1,
                    channel as u8 => 12, 3,
                    fsr as u8 => 9, 3,
                    0 => 8, 1,
                });

                let config_lsb = encode_reg8!({
                    data_rate as u8 => 5, 3,
                    mode as u8 => 4,1,
                    self.pull_up_enable as u8 => 3,1,
                    0b01 => 1,2,
                    1 => 0,1,
                });

                self.write_register(config_msb, config_lsb).await?;

                let mut sum: i32 = 0;

                for _ in 0..sample_count {
                    self.int.wait_for_falling_edge().await;
                    let raw_data = self.read_register().await?;
                    sum += i16::from_le_bytes(raw_data) as i32;
                }
                let average = (sum / sample_count as i32) as i16;
                Ok(average)
            }
            OperationMode::SingleShot => {
                let config_msb = encode_reg8!({
                    1 => 15, 1,
                    channel as u8 => 12, 3,
                    fsr as u8 => 9, 3,
                    1 => 8, 1,
                });

                let config_lsb = encode_reg8!({
                    data_rate as u8 => 5, 3,
                    mode as u8 => 4,1,
                    self.pull_up_enable as u8 => 3,1,
                    0b01 => 1,2,
                    1 => 0,1,
                });

                self.write_register(config_msb, config_lsb).await?;
                self.int.wait_for_falling_edge().await;
                let raw_data = self.read_register().await?;
                Ok(i16::from_le_bytes(raw_data))
            }
        }

    }
}
