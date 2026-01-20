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
    Config,
    adc::{Adc, AdcChannel, AdcConfig},
    bind_interrupts,
    can::{
        self, BufferedFdCanReceiver, BufferedFdCanSender, CanConfigurator, RxFdBuf, TxFdBuf,
        frame::FdFrame,
    },
    gpio::{Level, Output, Speed},
    i2c::{self, I2c, Master},
    mode::Async,
    peripherals::{self, FDCAN1, IWDG},
    rcc::{self, mux::Fdcansel},
    time::khz,
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, DynamicSender, Receiver, Sender},
    mutex::Mutex,
    watch::{DynReceiver, Watch},
};
use embassy_time::Timer;
use south_common::{
    TMValue, TelemetryContainer, TelemetryDefinition, can_config::CanPeriphConfig, telecommands,
    telemetry::eps as tm, telemetry_container, types::Telecommand,
};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod adc_driver;


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

#[embassy_executor::main]
async fn main(spawner: Spawner) {

}
