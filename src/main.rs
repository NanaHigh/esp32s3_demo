use esp_idf_svc as svc;
use hal::sys;
use svc::hal;

use esp32s3_demo::lcd::{LcdConfig, LcdDriver, LcdDriverType, LcdPins};
use esp32s3_demo::motor::{MotorCommand, MotorDriver, MotorSelect};
use esp32s3_demo::sensor::{I2cSensorDriver, UartSensorDriver};

use hal::delay::FreeRtos;
use hal::gpio::{AnyIOPin, PinDriver, Pull};
use hal::i2c::{I2cConfig, I2cDriver};
use hal::ledc::{self, LedcDriver, LedcTimerDriver};
use hal::peripherals::Peripherals;
use hal::prelude::*;
use hal::spi::Dma;
use hal::uart::{self, UartDriver};

use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{renderer::CharacterStyle, Text};

fn main() -> anyhow::Result<()> {
    sys::link_patches();

    svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    let i2c0_scl = p.pins.gpio4;
    let i2c0_sda = p.pins.gpio5;
    let uart1_tx = p.pins.gpio40;
    let uart1_rx = p.pins.gpio39;

    let uart1_config = uart::config::Config::default().baudrate(Hertz(115_200));
    let uart1 = UartDriver::new(
        p.uart1,
        uart1_tx,
        uart1_rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart1_config,
    )?;
    let mut uart1_sensor = UartSensorDriver::new(uart1)?;

    let i2c0_cfg = I2cConfig::new()
        .baudrate(400.kHz().into())
        .scl_enable_pullup(true)
        .sda_enable_pullup(true);
    let i2c0 = I2cDriver::new(p.i2c0, i2c0_sda, i2c0_scl, &i2c0_cfg)?;
    let mut i2c0_sensor = I2cSensorDriver::new(i2c0)?;

    let pwm_a_pin = p.pins.gpio1;
    let a_in1 = PinDriver::output(p.pins.gpio13)?;
    let a_in2 = PinDriver::output(p.pins.gpio14)?;
    let standby = PinDriver::output(p.pins.gpio12)?;
    let pwm_b_pin = p.pins.gpio9;
    let b_in1 = PinDriver::output(p.pins.gpio11)?;
    let b_in2 = PinDriver::output(p.pins.gpio10)?;

    let pwm_tim = LedcTimerDriver::new(
        p.ledc.timer0,
        &ledc::config::TimerConfig::default().frequency(10.kHz().into()),
    )?;
    let pwm_a = LedcDriver::new(p.ledc.channel0, &pwm_tim, pwm_a_pin)?;
    let pwm_b = LedcDriver::new(p.ledc.channel1, &pwm_tim, pwm_b_pin)?;
    let mut motor = MotorDriver::new(pwm_a, a_in1, a_in2, standby, pwm_b, b_in1, b_in2)?;

    let rst = p.pins.gpio17.into();
    let dc = p.pins.gpio18.into();
    let mut backlight = PinDriver::output(p.pins.gpio8)?;
    let sclk = p.pins.gpio6.into();
    let sdo = p.pins.gpio7.into();
    // Pin sdi and cs is not used.
    let sdi = p.pins.gpio19.into();
    let cs = p.pins.gpio20.into();

    // Dma size is 32KB.
    let lcd_cfg = LcdConfig::new(LcdDriverType::St7789, 240, 240).dma(Dma::Auto(32 * 1024))?;

    let lcd_pins = LcdPins {
        sclk,
        sdo,
        sdi,
        rst,
        dc,
        cs,
    };

    let mut display = LcdDriver::new(p.spi2, lcd_cfg, lcd_pins, 80.MHz().into())?.as_st7789()?;

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

    let mut buzzer = PinDriver::output(p.pins.gpio45)?;
    buzzer.set_high().ok();
    let mut sw1 = PinDriver::input(p.pins.gpio38)?;
    sw1.set_pull(Pull::Up).ok();

    log::info!("Wait for initialization...");
    FreeRtos::delay_ms(3000);
    buzzer.set_low().ok();

    // Test the buzzer and motor.
    motor.set_speed(MotorSelect::A, MotorCommand::Forward, 0.8)?;
    motor.set_speed(MotorSelect::B, MotorCommand::Forward, 0.8)?;
    FreeRtos::delay_ms(3000);
    buzzer.set_high().ok();
    motor.set_speed(MotorSelect::A, MotorCommand::Stop, 0.0)?;
    motor.set_speed(MotorSelect::B, MotorCommand::Stop, 0.0)?;

    let mut num = 0u8;
    let mut max_tries = 100;
    while max_tries > 0 {
        if let Ok(label) = uart1_sensor.read_cam_label() {
            num = label;
        }

        if num != 0 {
            let lcd_string = format!("Detected: {}", num);

            Text::new(&lcd_string, Point::new(0, 120), text_style)
                .draw(&mut display)
                .unwrap();
        }

        max_tries -= 1;

        FreeRtos::delay_ms(20);
    }

    log::info!("Location is {num}");

    while sw1.is_high() {}

    log::info!("Start");

    loop {
        let angle = i2c0_sensor.read_angle()?;
        let line = i2c0_sensor.read_line()?;

        log::info!("Line: {:?}", line);

        let lcd_string = format!(
            "{roll_label:<8}{roll:>8.2}\n\
             {pitch_label:<8}{pitch:>8.2}\n\
             {yaw_label:<8}{yaw:>8.2}",
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
}
