#![no_std]
#![no_main]
#![feature(variant_count)]
#![feature(type_alias_impl_trait)]
#![feature(iter_collect_into)]
#![feature(iterator_try_collect)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)] // This feature is incomplete but beeing used in a benign context

use defmt::*;

use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{
        self, BufferedFdCanReceiver, BufferedFdCanSender, CanConfigurator, RxFdBuf, TxFdBuf,
        frame::FdFrame,
    },
    exti::{ExtiInput, InterruptHandler},
    gpio::{Level, Output, Pull, Speed},
    interrupt::typelevel::EXTI4_15,
    mode::Async,
    peripherals::{FDCAN1, IWDG},
    rcc::{self, mux::Fdcansel},
    spi::{self, Spi, mode::Master},
    time::Hertz,
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, DynamicSender, Receiver, Sender},
    mutex::Mutex,
};
use embassy_time::Timer;
use south_common::{
    TMValue, TelemetryContainer, TelemetryDefinition, can_config::CanPeriphConfig, telecommands,
    telemetry::lower_sensor as tm, telemetry_container, types::Telecommand,
};
use static_cell::StaticCell;

use crate::adc_driver::{
    Adc as SensorAdc, DataRate, OperationMode,
};

use {defmt_rtt as _, panic_probe as _};

mod adc_driver;

// bind interrupts
bind_interrupts!(struct Irqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;

    EXTI4_15 => InterruptHandler<EXTI4_15>;
});

/// config rcc for higher sysclock and fdcan periph clock to make sure
/// all messages can be received without package drop
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::Hsi {
        sys_div: rcc::HsiSysDiv::DIV1,
    });
    rcc_config.sys = rcc::Sysclk::PLL1_R;
    rcc_config.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV1,
        mul: rcc::PllMul::MUL8,
        divp: None,
        divq: Some(rcc::PllQDiv::DIV2),
        divr: Some(rcc::PllRDiv::DIV2),
    });
    rcc_config.mux.fdcansel = Fdcansel::PLL1_Q;
    rcc_config
}

// General setup stuff
const STARTUP_DELAY: u64 = 300;

const WATCHDOG_TIMEOUT_US: u32 = 300_000;
const WATCHDOG_PETTING_INTERVAL_US: u32 = WATCHDOG_TIMEOUT_US / 2;

// Telemtry container
type LowerSensorTMContainer = telemetry_container!(tm);

const TM_CHANNEL_BUF_SIZE: usize = 5;
const CMD_CHANNEL_BUF_SIZE: usize = 5;

static TMC: StaticCell<Channel<ThreadModeRawMutex, LowerSensorTMContainer, TM_CHANNEL_BUF_SIZE>> =
    StaticCell::new();
static CMDC: StaticCell<Channel<ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>> =
    StaticCell::new();

// static paripherals
static SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Master>>> = StaticCell::new();

// CAN configuration
const RX_BUF_SIZE: usize = 64;
const TX_BUF_SIZE: usize = 64;

static RX_BUF: StaticCell<RxFdBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<TxFdBuf<TX_BUF_SIZE>> = StaticCell::new();

#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG>) {
    loop {
        watchdog.pet();
        Timer::after_micros(WATCHDOG_PETTING_INTERVAL_US.into()).await;
    }
}

// tm sending task
#[embassy_executor::task]
pub async fn tm_thread(
    mut can_sender: BufferedFdCanSender,
    tm_channel: Receiver<'static, ThreadModeRawMutex, LowerSensorTMContainer, TM_CHANNEL_BUF_SIZE>,
) {
    loop {
        let container = tm_channel.receive().await;
        match FdFrame::new_standard(container.id(), container.bytes()) {
            Ok(frame) => {
                can_sender.write(frame).await;
            }
            Err(e) => error!("error constructing can message: {}", e),
        }
    }
}

// tc receiving task
#[embassy_executor::task]
pub async fn tc_thread(
    can_receiver: BufferedFdCanReceiver,
    tc_channel: Sender<'static, ThreadModeRawMutex, Telecommand, TM_CHANNEL_BUF_SIZE>,
) {
    loop {
        match can_receiver.receive().await {
            Ok(envelope) => match Telecommand::read(envelope.frame.data()) {
                Ok(cmd) => tc_channel.send(cmd.1).await,
                Err(_) => error!("error parsing tc"),
            },
            Err(e) => error!("error in frame! {}", e),
        }
    }
}


// adc task
#[embassy_executor::task]
pub async fn adc_thread(
    tm_sender: DynamicSender<'static, LowerSensorTMContainer>,
    mut adc: SensorAdc<'static>,
    pressure1_def: &'static dyn TelemetryDefinition,
    pressure2_def: &'static dyn TelemetryDefinition,
    temp_def: &'static dyn TelemetryDefinition,
    temp_adc_def: &'static dyn TelemetryDefinition,
) {
    loop {
        match adc
            .read_all_channels(DataRate::SPS64, OperationMode::Continuous(4), true)
            .await
        {
            Ok((sensor_data, temp_adc)) => {
                let container_p1 =
                    LowerSensorTMContainer::new(pressure1_def, &sensor_data[0]).unwrap();
                tm_sender.send(container_p1).await;
                let container_p2 =
                    LowerSensorTMContainer::new(pressure2_def, &sensor_data[1]).unwrap();
                tm_sender.send(container_p2).await;
                let container_temp =
                    LowerSensorTMContainer::new(temp_def, &sensor_data[2]).unwrap();
                tm_sender.send(container_temp).await;
                let container_temp_adc =
                    LowerSensorTMContainer::new(temp_adc_def, &temp_adc).unwrap();
                tm_sender.send(container_temp_adc).await;
            }
            Err(e) => error!("could not read sensor data from adc {}",e),
        }
    }
}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    info!("Launching");

    // create independent watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US);

    // TM channel setup
    let tm_channel = TMC.init(Channel::new());
    let cmd_channel = CMDC.init(Channel::new());

    let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);

    // -- CAN configuration
    let mut can_configurator =
        CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs));

    can_configurator
        .add_receive_topic(telecommands::Telecommand.id())
        .unwrap();

    let can_interface = can_configurator.activate(
        TX_BUF.init(TxFdBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxFdBuf::<RX_BUF_SIZE>::new()),
    );

    // Spi / ADC setup
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(3_000_000);
    spi_config.gpio_speed = Speed::High;
    spi_config.mode = spi::MODE_1;

    let cs_adc = Output::new(p.PA8, Level::High, Speed::Low);
    let int = ExtiInput::new(p.PA4, p.EXTI4, Pull::None, Irqs);

    let spi = SPI.init(Mutex::new(Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH1, p.DMA1_CH2, spi_config,
    )));
    let adc = SensorAdc::new(spi, cs_adc, int);

    // Thread spawning
    watchdog.unleash();
    spawner.must_spawn(petter(watchdog));

    Timer::after_millis(STARTUP_DELAY).await;

    spawner.must_spawn(adc_thread(
        tm_channel.dyn_sender(),
        adc,
        &tm::Pressure1,
        &tm::Pressure2,
        &tm::Temp,
        &tm::AdcTemp,
    ));

    spawner.must_spawn(tm_thread(can_interface.writer(), tm_channel.receiver()));
    spawner.must_spawn(tc_thread(can_interface.reader(), cmd_channel.sender()));
}
