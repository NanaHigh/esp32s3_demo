// TODO: improve code abstraction.
#![allow(clippy::too_many_arguments)]
use esp_idf_svc::hal;

use crate::pid::SpeedPidController;
use hal::delay::FreeRtos;
use hal::gpio::{Output, OutputPin, PinDriver};
use hal::ledc::LedcDriver;
use hal::pcnt::PcntDriver;
use hal::sys::EspError;
use thiserror::Error;

const KP: f32 = 1.3; // Proportional coefficient
const KI: f32 = 0.01; // Integral coefficient
const KD: f32 = 0.15; // Derivative coefficient

/// Motor error.
#[derive(Debug, Error)]
pub enum MotorError {
    #[error("Pwm error: {0}")]
    PwmError(EspError),
    #[error("Encoder error: {0}")]
    EncoderError(EspError),
}

/// Motor driver.
pub struct MotorDriver<'a, AIN1, AIN2, STBY, BIN1, BIN2>
where
    AIN1: OutputPin,
    AIN2: OutputPin,
    STBY: OutputPin,
    BIN1: OutputPin,
    BIN2: OutputPin,
{
    pwm_a: LedcDriver<'a>,
    a_in1: PinDriver<'a, AIN1, Output>,
    a_in2: PinDriver<'a, AIN2, Output>,
    standby: PinDriver<'a, STBY, Output>,
    pwm_b: LedcDriver<'a>,
    b_in1: PinDriver<'a, BIN1, Output>,
    b_in2: PinDriver<'a, BIN2, Output>,
    encoder_a: PcntDriver<'a>,
    encoder_b: PcntDriver<'a>,
    pid_a: SpeedPidController,
    pid_b: SpeedPidController,
    speed_a: f32,
    speed_b: f32,
    pwm_val_a: f32,
    pwm_val_b: f32,
}

impl<'a, AIN1, AIN2, STBY, BIN1, BIN2> MotorDriver<'a, AIN1, AIN2, STBY, BIN1, BIN2>
where
    AIN1: OutputPin,
    AIN2: OutputPin,
    STBY: OutputPin,
    BIN1: OutputPin,
    BIN2: OutputPin,
{
    /// Create a new Motor driver.
    pub fn new(
        pwm_a: LedcDriver<'a>,
        a_in1: PinDriver<'a, AIN1, Output>,
        a_in2: PinDriver<'a, AIN2, Output>,
        standby: PinDriver<'a, STBY, Output>,
        pwm_b: LedcDriver<'a>,
        b_in1: PinDriver<'a, BIN1, Output>,
        b_in2: PinDriver<'a, BIN2, Output>,
        encoder_a: PcntDriver<'a>,
        encoder_b: PcntDriver<'a>,
    ) -> Result<Self, MotorError> {
        Ok(Self {
            pwm_a,
            a_in1,
            a_in2,
            standby,
            pwm_b,
            b_in1,
            b_in2,
            encoder_a,
            encoder_b,
            pid_a: SpeedPidController::new(KP, KI, KD),
            pid_b: SpeedPidController::new(KP, KI, KD),
            // Initialize the encoder values to 0
            speed_a: 0.0,
            speed_b: 0.0,
            pwm_val_a: 0.0,
            pwm_val_b: 0.0,
        })
    }

    /// Set the motor speed (Speed is -100.0 ~ 100.0).
    pub fn set_pwm_speed(&mut self, speed_a: f32, speed_b: f32) -> Result<(), MotorError> {
        let speed_a = speed_a.clamp(-100.0, 100.0);
        let speed_b = speed_b.clamp(-100.0, 100.0);

        if speed_a > 0.0 {
            self.a_in1.set_low().ok();
            self.a_in2.set_high().ok();
            self.pwm_a
                .set_duty(((speed_a / 100.0) * self.pwm_a.get_max_duty() as f32) as u32)
                .map_err(|err| MotorError::PwmError(err))?;
        } else {
            self.a_in1.set_high().ok();
            self.a_in2.set_low().ok();
            self.pwm_a
                .set_duty(((-speed_a / 100.0) * self.pwm_a.get_max_duty() as f32) as u32)
                .map_err(|err| MotorError::PwmError(err))?;
        }

        if speed_b > 0.0 {
            self.b_in1.set_low().ok();
            self.b_in2.set_high().ok();
            self.pwm_b
                .set_duty(((speed_b / 100.0) * self.pwm_a.get_max_duty() as f32) as u32)
                .map_err(|err| MotorError::PwmError(err))?;
        } else {
            self.b_in1.set_high().ok();
            self.b_in2.set_low().ok();
            self.pwm_b
                .set_duty(((-speed_b / 100.0) * self.pwm_a.get_max_duty() as f32) as u32)
                .map_err(|err| MotorError::PwmError(err))?;
        }
        self.standby.set_high().ok();

        Ok(())
    }

    /// Get the current PWM value.
    pub fn get_pwm_value(&self) -> Result<(f32, f32), MotorError> {
        Ok((self.pwm_val_a, self.pwm_val_b))
    }

    /// Stop the motor.
    pub fn stop(&mut self) -> Result<(), MotorError> {
        self.speed_a = 0.0;
        self.speed_b = 0.0;
        self.pwm_val_a = 0.0;
        self.pwm_val_b = 0.0;
        self.pid_a.set_target(0.0);
        self.pid_b.set_target(0.0);
        self.set_motor_speed(0.0, 0.0)?;
        self.standby.set_low().ok();
        Ok(())
    }

    /// Get the encoder value.
    pub fn get_counter_value(&mut self) -> Result<(f32, f32), MotorError> {
        Ok((
            self.encoder_a
                .get_counter_value()
                .map_err(MotorError::EncoderError)
                .map(|v| v as f32)?,
            self.encoder_b
                .get_counter_value()
                .map_err(MotorError::EncoderError)
                .map(|v| v as f32)?,
        ))
    }

    /// Get the real-time speed of the motor.
    pub fn get_motor_speed(&mut self) -> Result<(f32, f32), MotorError> {
        const SAMPLE_COUNT: u8 = 3; // Sampling count
        let mut speed_a_sum = 0.0;
        let mut speed_b_sum = 0.0;

        // Take multiple samples and average
        for _ in 0..SAMPLE_COUNT {
            self.encoder_a
                .counter_clear()
                .map_err(|err| MotorError::EncoderError(err))?;
            self.encoder_b
                .counter_clear()
                .map_err(|err| MotorError::EncoderError(err))?;

            let (last_a_val, last_b_val) = self.get_counter_value()?;

            // Set sampling time to 1ms to improve accuracy
            FreeRtos::delay_ms(1);

            let (current_a_val, current_b_val) = self.get_counter_value()?;

            // Calculate single speed
            speed_a_sum += (current_a_val - last_a_val) * 1000.0 / (20.0 * 13.0);
            speed_b_sum += (current_b_val - last_b_val) * 1000.0 / (20.0 * 13.0);
        }

        // Calculate average speed
        let new_speed_a = speed_a_sum / SAMPLE_COUNT as f32;
        let new_speed_b = speed_b_sum / SAMPLE_COUNT as f32;

        // Use low-pass filtering to smooth speed
        const ALPHA: f32 = 0.7; // Filtering coefficient
        self.speed_a = ALPHA * self.speed_a + (1.0 - ALPHA) * new_speed_a;
        self.speed_b = ALPHA * self.speed_b + (1.0 - ALPHA) * new_speed_b;

        Ok((self.speed_a, self.speed_b))
    }

    /// Set the motor speed.
    pub fn set_motor_speed(
        &mut self,
        target_a_speed: f32,
        target_b_speed: f32,
    ) -> Result<(), MotorError> {
        // Set target speed
        self.pid_a.set_target(target_a_speed);
        self.pid_b.set_target(target_b_speed);

        // Get current speed
        let (current_a_speed, current_b_speed) = self.get_motor_speed()?;

        // Calculate PID output
        let pwm_adjust_a = self.pid_a.calculate(current_a_speed);
        let pwm_adjust_b = self.pid_b.calculate(current_b_speed);

        // Adjust PWM values
        self.pwm_val_a = (self.pwm_val_a + pwm_adjust_a).clamp(-100.0, 100.0);
        self.pwm_val_b = (self.pwm_val_b + pwm_adjust_b).clamp(-100.0, 100.0);
        self.set_pwm_speed(self.pwm_val_a, self.pwm_val_b)?;

        Ok(())
    }
}
