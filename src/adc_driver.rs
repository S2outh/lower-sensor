use embassy_stm32::{
    exti::ExtiInput,
    gpio::Output,
    mode::Async,
    spi::{Spi, mode::Master},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

type SpiError = embassy_stm32::spi::Error;

const OFFSET_DRIFT: f32 = 0.002;

/*
Für Kalibrierung später
- offset error
- gain error

-> (Wert - offset) * (1+ gain*1e-6)

let calibrated_data = match mode {
                    SensorMode::ADC => (average as f32 - offset) * (1.0 + (gain * 1e-6)),
                    SensorMode::Temp => {
                        let code_14bit = average >> 2;
                        // sign = MSB 0 or 1
                        if (code_14bit & 0x2000) == 0 {
                            // positiv
                            (code_14bit as f32) * 0.03125
                        } else {
                            //negative: 14-bit two's complement calculation
                            let mag = ((!code_14bit & 0x3FFF) + 1) as u16;
                            -(mag as f32) * 0.03125
                        }
                    }
                };
*/

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
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum SensorMode {
    ADC = 0,
    Temp = 1,
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum OperationMode {
    SingleShot = 1,
    Continuous(u16) = 0,
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum FSR {
    FSR6_144V = 0b000,
    FSR4_096V = 0b001,
    FSR2_048V = 0b010,
    FSR1_024V = 0b011,
    FSR0_512V = 0b100,
    FSR0_256V = 0b101,
}

impl FSR {
    pub fn get_gain_drift(&self) -> f32 {
        match self {
            FSR::FSR0_256V => 7.0,
            _ => 5.0,
        }
    }
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
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum TempCorrection {
    True(f32),
    False,
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum Channel {
    CH1 = 0b100,
    CH2 = 0b101,
    CH3 = 0b110,
}

impl Channel {
    pub fn get_channel(&self) -> usize {
        match self {
            Channel::CH1 => 0,
            Channel::CH2 => 1,
            Channel::CH3 => 2,
        }
    }
}

pub struct Adc<'d> {
    spi: &'d Mutex<ThreadModeRawMutex, Spi<'d, Async, Master>>,
    cs: Output<'d>,
    int: ExtiInput<'d>,
    pub pull_up_enable: bool,
}

impl<'d> Adc<'d> {
    pub fn new(
        spi: &'d Mutex<ThreadModeRawMutex, Spi<'d, Async, Master>>,
        cs: Output<'d>,
        int: ExtiInput<'d>,
    ) -> Self {
        Self {
            spi,
            cs,
            int,
            pull_up_enable: false,
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
                    0 => 7, 1,
                    channel as u8 => 4, 3,
                    fsr as u8 => 1, 3,
                    0 => 0, 1,
                });

                let config_lsb = encode_reg8!({
                    data_rate as u8 => 5, 3,
                    mode.clone() as u8 => 4,1,
                    self.pull_up_enable as u8 => 3,1,
                    0b01 => 1,2,
                    1 => 0,1,
                });

                let sleep_config_msb = encode_reg8!(base: config_msb, {
                    0 => 15, 1, // SS = 0
                    1 => 8, 1,  // MODE = 1 (Single-Shot / Power-Down)
                });

                self.write_register(config_msb, config_lsb).await?;

                let mut sum: i32 = 0;

                for _ in 0..sample_count {
                    self.cs.set_low();
                    self.int.wait_for_falling_edge().await;
                    let raw_data = self.read_register().await?;
                    sum += i16::from_le_bytes(raw_data) as i32;
                }
                let average = (sum / sample_count as i32) as i16;

                self.write_register(sleep_config_msb, config_lsb).await?;

                Ok(average)
            }
            OperationMode::SingleShot => {
                let config_msb = encode_reg8!({
                    1 => 7, 1,
                    channel as u8 => 4, 3,
                    fsr as u8 => 1, 3,
                    1 => 0, 1,
                });

                let config_lsb = encode_reg8!({
                    data_rate as u8 => 5, 3,
                    mode.clone() as u8 => 4,1,
                    self.pull_up_enable as u8 => 3,1,
                    0b01 => 1,2,
                    1 => 0,1,
                });

                self.write_register(config_msb, config_lsb).await?;
                self.cs.set_low();
                self.int.wait_for_falling_edge().await;
                let raw_data = self.read_register().await?;

                Ok(i16::from_le_bytes(raw_data))
            }
        }
    }

    pub async fn read_channel(
        &mut self,
        channel: Channel,
        data_rate: DataRate,
        mode: OperationMode,
    ) -> Result<i16, ErrorAdc> {
        match channel {
            // Pressure 1 / Pressure 2
            Channel::CH1 | Channel::CH2 => {
                let data = self
                    .read_data_adc(
                        channel,
                        FSR::FSR6_144V,
                        data_rate,
                        SensorMode::ADC,
                        mode,
                    )
                    .await?;
                Ok(data)
            }
            // Temp
            Channel::CH3 => {
                let data = self
                    .read_data_adc(
                        channel,
                        FSR::FSR2_048V,
                        data_rate,
                        SensorMode::ADC,
                        mode,
                    )
                    .await?;
                Ok(data)
            }
        }
    }

    pub async fn read_temp_adc(
        &mut self,
        data_rate: DataRate,
        mode: OperationMode,
    ) -> Result<i16, ErrorAdc> {
        let temp_data = self
            .read_data_adc(
                Channel::CH1,
                FSR::FSR1_024V,
                data_rate,
                SensorMode::Temp,
                mode,
            )
            .await?;
        Ok(temp_data)
    }

    pub async fn read_all_channels(
        &mut self,
        data_rate: DataRate,
        mode: OperationMode,
    ) -> Result<([i16; 3], i16), ErrorAdc>
where {
        let temp_adc = self.read_temp_adc(data_rate, mode).await?;

        let pressure_1 = self
            .read_data_adc(
                Channel::CH1,
                FSR::FSR6_144V,
                data_rate,
                SensorMode::ADC,
                mode,
            )
            .await?;
        let pressure_2 = self
            .read_data_adc(
                Channel::CH2,
                FSR::FSR6_144V,
                data_rate,
                SensorMode::ADC,
                mode,
            )
            .await?;
        let temp = self
            .read_data_adc(
                Channel::CH3,
                FSR::FSR2_048V,
                data_rate,
                SensorMode::ADC,
                mode,
            )
            .await?;

        Ok(([pressure_1, pressure_2, temp], temp_adc))
    }
}
