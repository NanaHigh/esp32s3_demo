use esp_idf_svc::hal;

use hal::gpio::{Output, OutputPin, PinDriver};
use hal::ledc::LedcDriver;
use thiserror::Error;

/// Motor error.
#[derive(Debug, Error)]
pub enum MotorError {
    #[error("Motor error")]
    MotorError,
}

/// Motor command.
#[derive(Debug, Clone, Copy)]
pub enum MotorCommand {
    /// Move forward.
    Forward,
    /// Move backward.
    Backward,
    /// Stop the motor.
    Stop,
    /// Standby mode.
    Standby,
}

/// Motor selector.
#[derive(Debug, Clone, Copy)]
pub enum MotorSelect {
    /// Select motor A.
    A,
    /// Select motor B.
    B,
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
    ) -> Result<Self, MotorError> {
        Ok(Self {
            pwm_a,
            a_in1,
            a_in2,
            standby,
            pwm_b,
            b_in1,
            b_in2,
        })
    }

    /// Set the motor speed.
    pub fn set_speed(
        &mut self,
        motor: MotorSelect,
        cmd: MotorCommand,
        speed: f32,
    ) -> Result<(), MotorError> {
        match motor {
            MotorSelect::A => {
                match cmd {
                    MotorCommand::Forward => {
                        self.a_in1.set_high().ok();
                        self.a_in2.set_low().ok();
                        self.standby.set_high().ok();
                    }
                    MotorCommand::Backward => {
                        self.a_in1.set_low().ok();
                        self.a_in2.set_high().ok();
                        self.standby.set_high().ok();
                    }
                    MotorCommand::Stop => {
                        self.a_in1.set_low().ok();
                        self.a_in2.set_low().ok();
                        self.standby.set_high().ok();
                    }
                    MotorCommand::Standby => {
                        self.standby.set_low().ok();
                    }
                }
                self.pwm_a
                    .set_duty((speed * (self.pwm_a.get_max_duty() as f32)) as u32)
                    .ok();
            }
            MotorSelect::B => {
                match cmd {
                    MotorCommand::Forward => {
                        self.b_in1.set_high().ok();
                        self.b_in2.set_low().ok();
                        self.standby.set_high().ok();
                    }
                    MotorCommand::Backward => {
                        self.b_in1.set_low().ok();
                        self.b_in2.set_high().ok();
                        self.standby.set_high().ok();
                    }
                    MotorCommand::Stop => {
                        self.b_in1.set_low().ok();
                        self.b_in2.set_low().ok();
                        self.standby.set_high().ok();
                    }
                    MotorCommand::Standby => {
                        self.standby.set_low().ok();
                    }
                }
                self.pwm_b
                    .set_duty((speed * (self.pwm_b.get_max_duty() as f32)) as u32)
                    .ok();
            }
        }
        Ok(())
    }
}
