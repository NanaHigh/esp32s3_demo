#[allow(unused_imports)]
use esp_idf_svc as svc;
use hal::sys;
use svc::hal;

use esp32s3_demo::sensor::{AsyncUartSensorDriver, I2cSensorDriver, UartSensorDriver};
use hal::delay::FreeRtos;
use hal::gpio::AnyIOPin;
use hal::i2c::{I2cConfig, I2cDriver};
use hal::peripherals::Peripherals;
use hal::prelude::*;
use hal::task::block_on;
use hal::uart::{self, AsyncUartDriver, UartDriver};

fn main() -> anyhow::Result<()> {
    sys::link_patches();

    svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    let i2c0_scl = p.pins.gpio14;
    let i2c0_sda = p.pins.gpio13;
    let uart1_tx = p.pins.gpio12;
    let uart1_rx = p.pins.gpio11;

    let uart1_config = uart::config::Config::default().baudrate(Hertz(115_200));
    let uart1 = AsyncUartDriver::new(
        p.uart1,
        uart1_tx,
        uart1_rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart1_config,
    )?;
    let mut uart1_sensor = AsyncUartSensorDriver::new(uart1);

    let i2c0_cfg = I2cConfig::new()
        .baudrate(400.kHz().into())
        .scl_enable_pullup(true)
        .sda_enable_pullup(true);
    let i2c0 = I2cDriver::new(p.i2c1, i2c0_sda, i2c0_scl, &i2c0_cfg)?;
    let mut i2c0_sensor = I2cSensorDriver::new(i2c0);

    block_on(async {
        loop {
            let angle = i2c0_sensor.read_angle()?;

            let distance = uart1_sensor.read_distance().await?;

            log::info!("Distance: {}mm", distance);
            log::info!("Roll: {:.2}", angle[0]);
            log::info!("Pitch: {:.2}", angle[1]);
            log::info!("Yaw: {:.2}", angle[2]);

            FreeRtos::delay_ms(2000);
        }
    })
}
