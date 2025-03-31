use esp_idf_svc::hal;

use hal::ledc::LedcDriver;
use thiserror::Error;

/// PWM error.
#[derive(Debug, Error)]
pub enum PwmError {
    #[error("Pwm error")]
    PwmError,
}

/// PWM driver.
pub struct PwmDriver<'a> {
    ledc: LedcDriver<'a>,
}

impl<'a> PwmDriver<'a> {
    /// Create a new PWM driver.
    pub fn new(ledc_drv: LedcDriver<'a>) -> Result<Self, PwmError> {
        Ok(Self { ledc: ledc_drv })
    }

    /// Set duty cycle.
    pub fn set_duty(&mut self, duty: u32) -> Result<(), PwmError> {
        self.ledc.set_duty(duty).map_err(|_| PwmError::PwmError)
    }

    /// Get max duty cycle.
    pub fn get_max_duty(&self) -> u32 {
        self.ledc.get_max_duty()
    }

    /// Set duty cycle percentage (0.0 ~ 1.0).
    pub fn set_duty_percent(&mut self, duty: f32) -> Result<(), PwmError> {
        self.set_duty((self.get_max_duty() as f32 * duty) as u32)
    }
}
