use esp_idf_svc as svc;
use hal::sys;
use svc::hal;

use esp32s3_demo::sensor::{I2cSensorDriver, UartSensorDriver};
use hal::delay::FreeRtos;
use hal::gpio::AnyIOPin;
use hal::i2c::{I2cConfig, I2cDriver};
use hal::peripherals::Peripherals;
use hal::prelude::*;
use hal::uart::{self, UartDriver};

fn main() -> anyhow::Result<()> {
    sys::link_patches();

    svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    let i2c0_scl = p.pins.gpio14;
    let i2c0_sda = p.pins.gpio13;
    let uart1_tx = p.pins.gpio12;
    let uart1_rx = p.pins.gpio11;

    let uart1_config = uart::config::Config::default().baudrate(Hertz(115_200));
    let uart1 = UartDriver::new(
        p.uart1,
        uart1_tx,
        uart1_rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart1_config,
    )?;
    let mut uart1_sensor = UartSensorDriver::new(uart1);

    let i2c0_cfg = I2cConfig::new()
        .baudrate(400.kHz().into())
        .scl_enable_pullup(true)
        .sda_enable_pullup(true);
    let i2c0 = I2cDriver::new(p.i2c1, i2c0_sda, i2c0_scl, &i2c0_cfg)?;
    let mut i2c0_sensor = I2cSensorDriver::new(i2c0);

    loop {
        let angle = &mut [0 as f32; 3];
        i2c0_sensor.read_angle(angle)?;

        let distance = &mut [0 as u16; 1];
        uart1_sensor.read_distance(distance)?;

        log::info!("Distance: {}mm", distance[0]);
        log::info!("Roll: {:.2}", angle[0]);
        log::info!("Pitch: {:.2}", angle[1]);
        log::info!("Yaw: {:.2}", angle[2]);

        FreeRtos::delay_ms(2000);
    }
}
