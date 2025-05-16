use esp_idf_svc::hal;

use crate::motor::{MotorDriver, MotorError};
use crate::pid::PositionPidController;
use crate::sensor::{I2cSensorDriver, I2cSensorError};
use hal::delay::FreeRtos;
use hal::gpio::{Output, OutputPin, PinDriver};

use thiserror::Error;

/// Car location.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CarLocation {
    Start,
    SingleLine,
    Crossing(u32),
    End,
}

/// Car driver error.
#[derive(Debug, Error)]
pub enum CarDriverError {
    #[error("Car motor error: {0}")]
    MotorError(#[from] MotorError),
    #[error("Car sensor error: {0}")]
    SensorError(#[from] I2cSensorError),
}

/// Car driver instance.
pub struct CarDriver<'a, AIN1, AIN2, STBY, BIN1, BIN2, BUZZER>
where
    AIN1: OutputPin,
    AIN2: OutputPin,
    STBY: OutputPin,
    BIN1: OutputPin,
    BIN2: OutputPin,
    BUZZER: OutputPin,
{
    motor: MotorDriver<'a, AIN1, AIN2, STBY, BIN1, BIN2>,
    i2c_sensor: I2cSensorDriver<'a>,
    buzzer: PinDriver<'a, BUZZER, Output>,
    car_last_location: CarLocation,
    pub car_current_location: CarLocation,
}

impl<'a, AIN1, AIN2, STBY, BIN1, BIN2, BUZZER> CarDriver<'a, AIN1, AIN2, STBY, BIN1, BIN2, BUZZER>
where
    AIN1: OutputPin,
    AIN2: OutputPin,
    STBY: OutputPin,
    BIN1: OutputPin,
    BIN2: OutputPin,
    BUZZER: OutputPin,
{
    /// Create a new car driver instance.
    pub fn new(
        motor: MotorDriver<'a, AIN1, AIN2, STBY, BIN1, BIN2>,
        i2c_sensor: I2cSensorDriver<'a>,
        buzzer: PinDriver<'a, BUZZER, Output>,
    ) -> Result<Self, CarDriverError> {
        Ok(Self {
            motor,
            i2c_sensor,
            buzzer,
            car_last_location: CarLocation::Start,
            car_current_location: CarLocation::Start,
        })
    }

    /// Make a sound for a specified number of times.
    pub fn beep(&mut self, times: u32) -> Result<(), CarDriverError> {
        for _ in 0..times {
            FreeRtos::delay_ms(100);
            self.buzzer.set_low().ok();
            FreeRtos::delay_ms(100);
            self.buzzer.set_high().ok();
        }
        Ok(())
    }

    /// Go straight for a specified ms.
    /// Resolution: 3ms.
    pub fn go_straight(&mut self, speed: f32, duration: u32) -> Result<(), CarDriverError> {
        let mut time = duration;
        const MEASURE_TIME_MS: u32 = 3; // Measure cycle 3ms

        // PID parameters
        const KP: f32 = 0.6; // Proportional coefficient
        const KI: f32 = 0.0; // Integral coefficient
        const KD: f32 = 0.1; // Derivative coefficient

        let mut error_sum = 0.0f32;
        let mut last_error = 0.0f32;

        while time > 0 {
            let line = self.i2c_sensor.read_line()?;

            // Calculate deviation value using finer weights
            let error = match line {
                // Centered
                [false, false, false, true, true, false, false, false]
                | [false, false, false, false, false, false, false, false] => 0.0,
                [false, false, false, true, false, false, false, false] => 0.002,
                [false, false, false, false, true, false, false, false] => -0.002,
                // Slightly to the right
                [false, false, true, false, false, false, false, false]
                | [false, false, true, true, false, false, false, false] => 0.3,
                // Slightly to the left
                [false, false, false, false, false, true, false, false]
                | [false, false, false, false, true, true, false, false] => -0.3,
                // Moderately to the right
                [false, true, false, false, false, false, false, false]
                | [false, true, true, false, false, false, false, false] => 0.45,
                // Moderately to the left
                [false, false, false, false, false, false, true, false]
                | [false, false, false, false, false, true, true, false] => -0.45,
                // Severely to the right
                [true, false, false, false, false, false, false, false]
                | [true, true, false, false, false, false, false, false] => 0.7,
                // Severely to the left
                [false, false, false, false, false, false, false, true]
                | [false, false, false, false, false, false, true, true] => -0.7,
                _ => last_error, // Maintain the last correction direction
            };

            // PID calculation
            let p = KP * error;
            error_sum = (error_sum + error).clamp(-10.0, 10.0); // Integral clamping
            let i = KI * error_sum;
            let d = KD * (error - last_error);
            last_error = error;

            // Calculate final output and limit the range
            let pid_output = (p + i + d).clamp(-1.0, 1.0);

            // Adjust left and right wheel speeds
            let mut speed_a = speed * (1.0 + pid_output);
            let mut speed_b = speed * (1.0 - pid_output);

            // Limit speed range
            speed_a = speed_a.clamp(speed * 0.5, speed * 1.5);
            speed_b = speed_b.clamp(speed * 0.5, speed * 1.5);

            // Set motor speed
            self.motor
                .set_motor_speed(speed_a, speed_b)
                .map_err(CarDriverError::MotorError)?;

            // Debug output
            log::info!(
                "Line: {:?}, Error: {:.2}, PID: {:.2}, SpeedA: {:.2}, SpeedB: {:.2}",
                line,
                error,
                pid_output,
                speed_a,
                speed_b
            );

            time -= MEASURE_TIME_MS;
        }

        self.motor.stop().map_err(CarDriverError::MotorError)?;
        Ok(())
    }

    /// Go straight until a condition is met.
    pub fn go_straight_until<F>(
        &mut self,
        speed: f32,
        condition: F,
        condition_met_time: u8,
    ) -> Result<(), CarDriverError>
    where
        F: Fn(&mut Self) -> bool,
    {
        // PID parameters
        const KP: f32 = 0.6; // Proportional coefficient
        const KI: f32 = 0.0; // Integral coefficient
        const KD: f32 = 0.1; // Derivative coefficient

        let mut error_sum = 0.0f32;
        let mut last_error = 0.0f32;

        let mut met_time = 0u8;

        while met_time < condition_met_time {
            let line = self.i2c_sensor.read_line()?;

            // Calculate deviation value using finer weights
            let error = match line {
                // Centered
                [false, false, false, true, true, false, false, false] => 0.0,
                // Very slightly to the right
                [false, false, false, true, false, false, false, false] => 0.002,
                // Very slightly to the left
                [false, false, false, false, true, false, false, false] => -0.002,
                // Slightly to the right
                [false, false, true, false, false, false, false, false]
                | [false, false, true, true, false, false, false, false] => 0.3,
                // Slightly to the left
                [false, false, false, false, false, true, false, false]
                | [false, false, false, false, true, true, false, false] => -0.3,
                // Moderately to the right
                [false, true, false, false, false, false, false, false]
                | [false, true, true, false, false, false, false, false] => 0.45,
                // Moderately to the left
                [false, false, false, false, false, false, true, false]
                | [false, false, false, false, false, true, true, false] => -0.45,
                // Severely to the right
                [true, false, false, false, false, false, false, false]
                | [true, true, false, false, false, false, false, false] => 0.7,
                // Severely to the left
                [false, false, false, false, false, false, false, true]
                | [false, false, false, false, false, false, true, true] => -0.7,
                _ => last_error, // Maintain the last correction direction
            };

            // PID calculation
            let p = KP * error;
            error_sum = (error_sum + error).clamp(-10.0, 10.0); // Integral clamping
            let i = KI * error_sum;
            let d = KD * (error - last_error);
            last_error = error;

            // Calculate final output and limit the range
            let pid_output = (p + i + d).clamp(-1.0, 1.0);

            // Adjust left and right wheel speeds
            let mut speed_a = speed * (1.0 + pid_output);
            let mut speed_b = speed * (1.0 - pid_output);

            // Limit speed range
            speed_a = speed_a.clamp(speed * 0.5, speed * 1.5);
            speed_b = speed_b.clamp(speed * 0.5, speed * 1.5);

            // Set motor speed
            self.motor
                .set_motor_speed(speed_a, speed_b)
                .map_err(CarDriverError::MotorError)?;

            if !condition(self) {
                met_time += 1;
                if condition_met_time > 1 {
                    FreeRtos::delay_ms(100);
                }
            }

            // Debug output
            log::info!(
                "Line: {:?}, Error: {:.2}, PID: {:.2}, SpeedA: {:.2}, SpeedB: {:.2}",
                line,
                error,
                pid_output,
                speed_a,
                speed_b
            );
        }

        self.motor.stop().map_err(CarDriverError::MotorError)?;
        Ok(())
    }

    /// Turn in place.
    /// Angle increment > 0 means turn left, < 0 means turn right.
    /// Suggested range: -180 to 180 degrees.
    pub fn turn_in_place(&mut self, angle_increment: f32) -> Result<(), CarDriverError> {
        // Create angle PID controller
        const KP: f32 = 0.75; // Proportional coefficient
        const KI: f32 = 0.05; // Integral coefficient
        const KD: f32 = 0.3; // Derivative coefficient
        let mut angle_pid = PositionPidController::new(KP, KI, KD);

        // Get initial angle
        self.i2c_sensor
            .reset_yaw_angle()
            .map_err(CarDriverError::SensorError)?;
        let angle = self.i2c_sensor.read_angle()?;
        let initial_angle = angle[2]; // Use yaw angle

        // Calculate target angle
        let target_angle = initial_angle + angle_increment;
        angle_pid.set_target(target_angle);

        const MEASURE_TIME_MS: u32 = 3; // Measure cycle 3ms
        const MAX_ADJUST_TIME: u32 = 2000; // Maximum adjustment time 4 seconds
        let mut adjust_time = 0;

        while adjust_time < MAX_ADJUST_TIME {
            let current_angle = self.i2c_sensor.read_angle()?[2];
            let pid_output = angle_pid.calculate(current_angle);

            // Dynamic base speed - adjust based on error magnitude
            let error = (target_angle - current_angle).abs();
            let base_speed = if error > 30.0 {
                4.0 // Reduce maximum speed
            } else if error > 15.0 {
                3.0
            } else if error > 5.0 {
                2.5
            } else {
                2.0 // Smoother low speed
            };

            // Optimize speed mapping
            let motor_speed = base_speed * (pid_output / 90.0).clamp(-6.0, 6.0);

            // Add deadzone control
            let final_speed = if motor_speed.abs() < 0.1 {
                0.0
            } else {
                motor_speed
            };

            self.motor.set_motor_speed(final_speed, -final_speed)?;

            // Debug output
            log::info!(
                "Target: {:.2}, Current: {:.2}, PID: {:.2}, Speed: {:.2}",
                target_angle,
                current_angle,
                pid_output,
                final_speed
            );

            // Check if target angle is reached
            if error < 1.0 {
                let mut stable_count = 0;
                for _ in 0..4 {
                    // Increase stability check count
                    FreeRtos::delay_ms(1);
                    let check_angle = self.i2c_sensor.read_angle()?[2];
                    if (check_angle - target_angle).abs() < 0.8 {
                        stable_count += 1;
                    }
                }
                if stable_count >= 3 {
                    // Allow one deviation
                    break;
                }
            }
            adjust_time += MEASURE_TIME_MS;
        }

        self.motor.stop()?;

        Ok(())
    }

    /// Car routing process.
    pub fn car_routing(&mut self, base_speed: f32, num: u8) -> Result<(), CarDriverError> {
        // TODO

        Ok(())
    }
}
