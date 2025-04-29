use esp_idf_svc as svc;
use hal::sys;
use svc::hal;

use esp32s3_demo::lcd::{LcdConfig, LcdDriver, LcdDriverType};
use esp32s3_demo::pwm::PwmDriver;
use esp32s3_demo::sensor::{AsyncUartSensorDriver, I2cSensorDriver};

use hal::delay::FreeRtos;
use hal::gpio::{AnyIOPin, PinDriver};
use hal::i2c::{I2cConfig, I2cDriver};
use hal::ledc::{self, LedcDriver, LedcTimerDriver};
use hal::peripherals::Peripherals;
use hal::prelude::*;
use hal::spi::Dma;
use hal::task::block_on;
use hal::uart::{self, AsyncUartDriver};

use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{renderer::CharacterStyle, Text};

fn main() -> anyhow::Result<()> {
    sys::link_patches();

    svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    let i2c0_scl = p.pins.gpio14;
    let i2c0_sda = p.pins.gpio13;
    let uart1_tx = p.pins.gpio12;
    let uart1_rx = p.pins.gpio11;
    let pwm0_pin = p.pins.gpio10;

    let uart1_config = uart::config::Config::default().baudrate(Hertz(115_200));
    let uart1 = AsyncUartDriver::new(
        p.uart1,
        uart1_tx,
        uart1_rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart1_config,
    )?;
    let mut uart1_sensor = AsyncUartSensorDriver::new(uart1)?;

    let i2c0_cfg = I2cConfig::new()
        .baudrate(400.kHz().into())
        .scl_enable_pullup(true)
        .sda_enable_pullup(true);
    let i2c0 = I2cDriver::new(p.i2c0, i2c0_sda, i2c0_scl, &i2c0_cfg)?;
    let mut i2c0_sensor = I2cSensorDriver::new(i2c0)?;

    let driver0 = LedcTimerDriver::new(
        p.ledc.timer0,
        &ledc::config::TimerConfig::default().frequency(100.kHz().into()),
    )?;
    let tim0_ch0 = LedcDriver::new(p.ledc.channel0, driver0, pwm0_pin)?;
    let mut pwm0 = PwmDriver::new(tim0_ch0)?;
    pwm0.set_duty_percent(0.12)?;

    let rst = p.pins.gpio3.into();
    let dc = p.pins.gpio4.into();
    let mut backlight = PinDriver::output(p.pins.gpio5)?;
    let sclk = p.pins.gpio6.into();
    let sdo = p.pins.gpio7.into();
    let sdi = p.pins.gpio8.into();
    let cs = p.pins.gpio9.into();

    // Dma size is 32KB.
    let lcd_cfg = LcdConfig::new(LcdDriverType::St7789, 240, 240).dma(Dma::Auto(32 * 1024))?;

    let mut display = LcdDriver::new(
        p.spi2,
        lcd_cfg,
        sclk,
        sdo,
        sdi,
        rst,
        dc,
        cs,
        80.MHz().into(),
    )?
    .as_st7789()?;

    let ferris = ImageRawLE::new(include_bytes!("./assets/ferris.raw"), 86);
    let mut text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    text_style.set_background_color(Some(Rgb565::BLACK));

    // Draw image on black background.
    // Turn on the backlight.
    display.clear(Rgb565::BLACK).unwrap();
    FreeRtos::delay_ms(50);
    backlight.set_high()?;
    Image::new(&ferris, Point::new(0, 0))
        .draw(&mut display)
        .unwrap();

    // Draw text below image.
    Text::new("Hello, Rust!\nI'm ferris!", Point::new(0, 80), text_style)
        .draw(&mut display)
        .unwrap();

    log::info!("Image printed!");

    block_on(async {
        loop {
            let angle = i2c0_sensor.read_angle()?;

            let distance = uart1_sensor.read_distance().await? as f32 / 10.0;

            let lcd_string = format!(
                "{label:<8}{value:>7.1}\n\
                 {roll_label:<8}{roll:>8.2}\n\
                 {pitch_label:<8}{pitch:>8.2}\n\
                 {yaw_label:<8}{yaw:>8.2}",
                label = "Distance:",
                value = distance,
                roll_label = "Roll:",
                roll = angle[0],
                pitch_label = "Pitch:",
                pitch = angle[1],
                yaw_label = "Yaw:",
                yaw = angle[2],
            );

            Text::new(&lcd_string, Point::new(0, 120), text_style)
                .draw(&mut display)
                .unwrap();
        }
    })
}
