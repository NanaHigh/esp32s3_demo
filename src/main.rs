use esp_idf_svc as svc;
use hal::sys;
use svc::hal;

use esp32s3_demo::car::CarDriver;
use esp32s3_demo::motor::MotorDriver;
use esp32s3_demo::sensor::{I2cSensorDriver, ImuMode, UartSensorDriver};
use hal::units::Hertz;

use hal::delay::FreeRtos;
use hal::gpio::{AnyIOPin, AnyInputPin, PinDriver, Pull};
use hal::i2c::{I2cConfig, I2cDriver};
use hal::ledc::{self, LedcDriver, LedcTimerDriver, Resolution};
use hal::pcnt::{
    PcntChannel, PcntChannelConfig, PcntControlMode, PcntCountMode, PcntDriver, PcntEvent, PinIndex,
};
use hal::peripherals::Peripherals;
use hal::uart::{self, UartDriver};

fn main() -> anyhow::Result<()> {
    sys::link_patches();

    svc::log::EspLogger::initialize_default();

    let p = Peripherals::take()?;
    let i2c0_scl = p.pins.gpio4;
    let i2c0_sda = p.pins.gpio5;
    let uart1_rx = p.pins.gpio39;
    // Not used.
    let uart1_tx = p.pins.gpio20;

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
        .baudrate(Hertz(400_000))
        .scl_enable_pullup(true)
        .sda_enable_pullup(true);
    let i2c0 = I2cDriver::new(p.i2c0, i2c0_sda, i2c0_scl, &i2c0_cfg)?;
    let mut i2c0_sensor = I2cSensorDriver::new(i2c0)?;
    i2c0_sensor.set_imu_mode(ImuMode::Axis6)?;

    let pwm_a_pin = p.pins.gpio1;
    let a_in1 = PinDriver::output(p.pins.gpio13)?;
    let a_in2 = PinDriver::output(p.pins.gpio14)?;
    let e1_a = p.pins.gpio40;
    let e1_b = p.pins.gpio41;
    let standby = PinDriver::output(p.pins.gpio12)?;
    let pwm_b_pin = p.pins.gpio9;
    let b_in1 = PinDriver::output(p.pins.gpio11)?;
    let b_in2 = PinDriver::output(p.pins.gpio10)?;
    let e2_a = p.pins.gpio42;
    let e2_b = p.pins.gpio2;

    let pwm_tim = LedcTimerDriver::new(
        p.ledc.timer0,
        &ledc::config::TimerConfig::default()
            .frequency(Hertz(3_000))
            .resolution(Resolution::Bits14),
    )?;
    let pwm_a = LedcDriver::new(p.ledc.channel0, &pwm_tim, pwm_a_pin)?;
    let pwm_b = LedcDriver::new(p.ledc.channel1, &pwm_tim, pwm_b_pin)?;
    let mut encoder_a = PcntDriver::new(
        p.pcnt0,
        Some(e1_a),
        Some(e1_b),
        Option::<AnyInputPin>::None,
        Option::<AnyInputPin>::None,
    )?;

    let mut encoder_b = PcntDriver::new(
        p.pcnt1,
        Some(e2_a),
        Some(e2_b),
        Option::<AnyInputPin>::None,
        Option::<AnyInputPin>::None,
    )?;

    encoder_a.channel_config(
        PcntChannel::Channel0,
        PinIndex::Pin0,
        PinIndex::Pin1,
        &PcntChannelConfig {
            lctrl_mode: PcntControlMode::Reverse,
            hctrl_mode: PcntControlMode::Keep,
            pos_mode: PcntCountMode::Increment, // Positive pulse increment
            neg_mode: PcntCountMode::Decrement, // Negative pulse decrement
            counter_h_lim: 32767,
            counter_l_lim: -32768,
        },
    )?;

    encoder_b.channel_config(
        PcntChannel::Channel0,
        PinIndex::Pin0,
        PinIndex::Pin1,
        &PcntChannelConfig {
            lctrl_mode: PcntControlMode::Reverse,
            hctrl_mode: PcntControlMode::Keep,
            pos_mode: PcntCountMode::Decrement, // Positive pulse decrement
            neg_mode: PcntCountMode::Increment, // Negative pulse increment
            counter_h_lim: 32767,
            counter_l_lim: -32768,
        },
    )?;

    encoder_a.set_filter_value(1000)?;
    encoder_a.filter_enable()?;
    encoder_a.event_enable(PcntEvent::HighLimit)?;
    encoder_a.event_enable(PcntEvent::LowLimit)?;
    encoder_a.counter_pause()?;
    encoder_a.counter_clear()?;
    encoder_a.counter_resume()?;

    encoder_b.set_filter_value(1000)?;
    encoder_b.filter_enable()?;
    encoder_b.event_enable(PcntEvent::HighLimit)?;
    encoder_b.event_enable(PcntEvent::LowLimit)?;
    encoder_b.counter_pause()?;
    encoder_b.counter_clear()?;
    encoder_b.counter_resume()?;

    let motor = MotorDriver::new(
        pwm_a, a_in1, a_in2, standby, pwm_b, b_in1, b_in2, encoder_a, encoder_b,
    )?;

    let mut buzzer = PinDriver::output(p.pins.gpio45)?;
    buzzer.set_high().ok();

    let mut car = CarDriver::new(motor, i2c0_sensor, buzzer)?;

    let mut sw1 = PinDriver::input(p.pins.gpio38)?;
    let mut sw2 = PinDriver::input(p.pins.gpio48)?;
    let mut sw3 = PinDriver::input(p.pins.gpio47)?;
    let mut sw4 = PinDriver::input(p.pins.gpio21)?;
    sw1.set_pull(Pull::Up).ok();
    sw2.set_pull(Pull::Up).ok();
    sw3.set_pull(Pull::Up).ok();
    sw4.set_pull(Pull::Up).ok();

    let mut num = 0u8;

    while sw1.is_high() {
        FreeRtos::delay_ms(100);
        if sw2.is_low() {
            FreeRtos::delay_ms(50);
            if sw2.is_low() {
                if num < 8 {
                    num += 1;
                    car.beep(1)?;
                }
            }
        }

        if sw3.is_low() {
            FreeRtos::delay_ms(50);
            if sw3.is_low() {
                if num > 0 {
                    num -= 1;
                    car.beep(1)?;
                }
            }
        }

        if sw4.is_low() {
            FreeRtos::delay_ms(50);
            if sw4.is_low() {
                let mut max_tries = 10;
                // Get stable camera label
                while max_tries > 0 {
                    if let Ok(label) = uart1_sensor.read_cam_label() {
                        num = label;
                        max_tries -= 1;
                    }

                    if num != 0 {
                        log::info!("Detected: {num}");
                    }

                    FreeRtos::delay_ms(20);
                }
            }
        }
    }

    log::info!("Location: {}", num);

    log::info!("Start...");

    let base_speed = 1.8; // Base speed
    car.car_routing_block(base_speed, num)?;

    log::info!("Car reached the end location!");

    Ok(())
}
