use esp_idf_svc::hal;

use crate::motor::{MotorDriver, MotorError};
use crate::pid::PositionPidController;
use crate::sensor::{I2cSensorDriver, I2cSensorError};
use hal::delay::FreeRtos;
use hal::gpio::{Output, OutputPin, PinDriver};

use thiserror::Error;

const TURN_ANGLE: f32 = 90.0;
const TURN_LEFT: f32 = TURN_ANGLE;
const TURN_RIGHT: f32 = -TURN_ANGLE;
const TURN_FORWARD_MS: u32 = 45;
const PARKING_TURN_FORWARD_MS: u32 = 100;
const BACK_FORWARD_MS: u32 = 550;
const PARKING_WAIT_TIME: u32 = 5000;
const START_END_WAIT_TIME: u32 = 5000;
const MEASURE_TIME_MS: u32 = 3;

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
    car_current_crossing_count: u32,
    crossing_detected: bool,
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
            car_current_crossing_count: 0,
            crossing_detected: false,
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
        let mut time = 0;

        // PID parameters
        const KP: f32 = 1.25; // Proportional coefficient
        const KI: f32 = 0.0; // Integral coefficient
        const KD: f32 = 0.05; // Derivative coefficient

        let mut error_sum = 0.0f32;
        let mut last_error = 0.0f32;

        while time < duration {
            let line = self.i2c_sensor.read_line()?;

            // Update location before processing line data
            self.update_location()?;
            log::info!("Current line pattern: {:?}", line);

            // Calculate deviation value using finer weights
            let error = match line {
                // Centered
                [false, false, false, true, true, false, false, false] => 0.0,
                // Very slightly to the right
                [false, false, false, true, false, false, false, false] => 0.03,
                // Very slightly to the left
                [false, false, false, false, true, false, false, false] => -0.03,
                // Slightly to the right
                [false, false, true, false, false, false, false, false]
                | [false, false, true, true, false, false, false, false] => 0.05,
                // Slightly to the left
                [false, false, false, false, false, true, false, false]
                | [false, false, false, false, true, true, false, false] => -0.05,
                // Moderately to the right
                [false, true, false, false, false, false, false, false]
                | [false, true, true, false, false, false, false, false] => 0.1,
                // Moderately to the left
                [false, false, false, false, false, false, true, false]
                | [false, false, false, false, false, true, true, false] => -0.1,
                // Severely to the right
                [true, false, false, false, false, false, false, false]
                | [true, true, false, false, false, false, false, false] => 0.2,
                // Severely to the left
                [false, false, false, false, false, false, false, true]
                | [false, false, false, false, false, false, true, true] => -0.2,
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
            let (min_speed, max_speed) = if speed >= 0.0 {
                (speed * 0.5, speed * 1.5)
            } else {
                (speed * 1.5, speed * 0.5)
            };
            speed_a = speed_a.clamp(min_speed, max_speed);
            speed_b = speed_b.clamp(min_speed, max_speed);

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

            time += MEASURE_TIME_MS;
        }

        self.motor.stop().map_err(CarDriverError::MotorError)?;
        Ok(())
    }

    /// Go straight until a condition is met.
    pub fn go_straight_until<F>(&mut self, speed: f32, condition: F) -> Result<(), CarDriverError>
    where
        F: Fn(&mut Self) -> bool,
    {
        // PID parameters
        const KP: f32 = 1.1; // Proportional coefficient
        const KI: f32 = 0.0; // Integral coefficient
        const KD: f32 = 0.05; // Derivative coefficient

        let mut error_sum = 0.0f32;
        let mut last_error = 0.0f32;

        while !condition(self) {
            let line = self.i2c_sensor.read_line()?;

            // Update location before processing line data
            self.update_location()?;
            log::info!("Current line pattern: {:?}", line);

            // Calculate deviation value using finer weights
            let error = match line {
                // Centered
                [false, false, false, true, true, false, false, false] => 0.0,
                // Very slightly to the right
                [false, false, false, true, false, false, false, false] => 0.03,
                // Very slightly to the left
                [false, false, false, false, true, false, false, false] => -0.03,
                // Slightly to the right
                [false, false, true, false, false, false, false, false]
                | [false, false, true, true, false, false, false, false] => 0.05,
                // Slightly to the left
                [false, false, false, false, false, true, false, false]
                | [false, false, false, false, true, true, false, false] => -0.05,
                // Moderately to the right
                [false, true, false, false, false, false, false, false]
                | [false, true, true, false, false, false, false, false] => 0.1,
                // Moderately to the left
                [false, false, false, false, false, false, true, false]
                | [false, false, false, false, false, true, true, false] => -0.1,
                // Severely to the right
                [true, false, false, false, false, false, false, false]
                | [true, true, false, false, false, false, false, false] => 0.2,
                // Severely to the left
                [false, false, false, false, false, false, false, true]
                | [false, false, false, false, false, false, true, true] => -0.2,
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
            let (min_speed, max_speed) = if speed >= 0.0 {
                (speed * 0.5, speed * 1.5)
            } else {
                (speed * 1.5, speed * 0.5)
            };
            speed_a = speed_a.clamp(min_speed, max_speed);
            speed_b = speed_b.clamp(min_speed, max_speed);

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
        }

        self.motor.stop().map_err(CarDriverError::MotorError)?;
        Ok(())
    }

    /// Go back in a straight line using motor speed feedback for correction.
    pub fn go_back(&mut self, speed: f32, duration: u32) -> Result<(), CarDriverError> {
        // PID parameters
        const KP: f32 = 0.8;
        const KI: f32 = 0.0;
        const KD: f32 = 0.05;

        let mut error_sum = 0.0f32;
        let mut last_error = 0.0f32;

        let mut time = 0;
        while time < duration {
            // Get real wheel speed
            let (real_a, real_b) = self
                .motor
                .get_motor_speed()
                .map_err(CarDriverError::MotorError)?;

            // Ensure the speeds is the same
            let error = real_a - real_b;

            let p = KP * error;
            error_sum = (error_sum + error).clamp(-10.0, 10.0);
            let i = KI * error_sum;
            let d = KD * (error - last_error);
            last_error = error;

            let pid_output = (p + i + d).clamp(-1.0, 1.0);

            let mut speed_a = speed - pid_output;
            let mut speed_b = speed + pid_output;

            let (min_speed, max_speed) = if speed < 0.0 {
                (speed * 1.5, speed * 0.5)
            } else {
                (speed * 0.5, speed * 1.5)
            };
            speed_a = speed_a.clamp(min_speed, max_speed);
            speed_b = speed_b.clamp(min_speed, max_speed);

            self.motor
                .set_motor_speed(speed_a, speed_b)
                .map_err(CarDriverError::MotorError)?;

            log::info!(
                "Back: RealA: {:.2}, RealB: {:.2}, Error: {:.2}, PID: {:.2}, SpeedA: {:.2}, SpeedB: {:.2}",
                real_a, real_b, error, pid_output, speed_a, speed_b
            );

            time += MEASURE_TIME_MS;
        }

        self.motor.stop().map_err(CarDriverError::MotorError)?;
        Ok(())
    }

    /// Turn.
    /// Angle increment > 0 means turn left, < 0 means turn right.
    /// Suggested range: -180 to 180 degrees.
    pub fn turn(&mut self, angle_increment: f32) -> Result<(), CarDriverError> {
        // Create angle PID controller
        const KP: f32 = 0.8; // Proportional coefficient
        const KI: f32 = 0.0; // Integral coefficient
        const KD: f32 = 0.08; // Derivative coefficient
        let mut angle_pid = PositionPidController::new(KP, KI, KD);

        // Get initial angle
        self.i2c_sensor
            .reset_yaw_angle()
            .map_err(CarDriverError::SensorError)?;
        FreeRtos::delay_ms(100);
        let angle = self.i2c_sensor.read_angle()?;
        let initial_angle = angle[2]; // Use yaw angle

        // Calculate target angle
        let target_angle = initial_angle + angle_increment;
        angle_pid.set_target(target_angle);

        const MAX_ADJUST_TIME: u32 = 1200; // Maximum adjustment time 1200 ms
        let mut adjust_time = 0;

        while adjust_time < MAX_ADJUST_TIME {
            let current_angle = self.i2c_sensor.read_angle()?[2];
            let pid_output = angle_pid.calculate(current_angle);

            // Dynamic base speed - adjust based on error magnitude
            let error = angle_diff(target_angle, current_angle).abs();
            let base_speed = if error > 30.0 {
                10.0 // Reduce maximum speed
            } else if error > 15.0 {
                8.0
            } else if error > 5.0 {
                6.0
            } else {
                5.0 // Smoother low speed
            };

            // Optimize speed mapping
            let motor_speed = base_speed * (pid_output / 90.0).clamp(-6.0, 6.0);

            // Add deadzone control
            let final_speed = if motor_speed.abs() < 0.1 {
                0.0
            } else {
                motor_speed
            };

            if angle_increment > 0.0 {
                // Left wheel stops right wheel moves
                self.motor.set_motor_speed(final_speed, 0.0)?;
            } else {
                // Right wheel stops, left moves
                self.motor.set_motor_speed(0.0, -final_speed)?;
            }

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
        const MAX_ADJUST_TIME: u32 = 1200; // Maximum adjustment time 1.2 seconds
        let mut adjust_time = 0;

        while adjust_time < MAX_ADJUST_TIME {
            let current_angle = self.i2c_sensor.read_angle()?[2];
            let pid_output = angle_pid.calculate(current_angle);

            // Dynamic base speed - adjust based on error magnitude
            let error = angle_diff(target_angle, current_angle).abs();
            let base_speed = if error > 30.0 {
                5.0 // Reduce maximum speed
            } else if error > 15.0 {
                4.0
            } else if error > 5.0 {
                2.5
            } else {
                2.0 // Smoother low speed
            };

            // Optimize speed mapping
            let motor_speed = base_speed * (pid_output / 90.0).clamp(-6.0, 6.0);

            // Add deadzone control
            let final_speed = if motor_speed.abs() < 0.5 {
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

    /// Update car location based on line sensor data.
    pub fn update_location(&mut self) -> Result<(), CarDriverError> {
        let line = self.i2c_sensor.read_line()?;

        // Store last location before updating
        if self.car_last_location != self.car_current_location {
            self.car_last_location = self.car_current_location;
            log::info!(
                "Location changed! Last location: {:?}",
                self.car_last_location
            );
        }

        // 判断是否为 crossing
        let is_crossing = match line {
            [true, true, true, _, _, _, _, _]
            | [_, true, true, true, _, _, _, _]
            | [_, _, true, true, true, _, _, _]
            | [_, _, _, true, true, true, _, _]
            | [_, _, _, _, true, true, true, _]
            | [_, _, _, _, _, true, true, true] => true,
            _ => false,
        };

        if is_crossing {
            if !self.crossing_detected {
                self.crossing_detected = true;
                self.car_current_crossing_count += 1;
                log::info!(
                    "Crossing detected! Count: {}",
                    self.car_current_crossing_count
                );
                self.car_current_location = CarLocation::Crossing(self.car_current_crossing_count);
                log::info!("Location updated to: {:?}", self.car_current_location);
            }
        } else {
            if self.crossing_detected {
                // Clear flag for next detection
                self.crossing_detected = false;
            }
            // Check if the car is on the single line
            let is_single_line = match line {
                [false, false, false, true, true, false, false, false]
                | [false, false, true, true, false, false, false, false]
                | [false, false, false, false, true, true, false, false]
                | [false, true, true, false, false, false, false, false]
                | [false, false, false, false, false, true, true, false]
                | [true, true, false, false, false, false, false, false]
                | [false, false, false, false, false, false, true, true]
                | [true, false, false, false, false, false, false, false]
                | [false, true, false, false, false, false, false, false]
                | [false, false, true, false, false, false, false, false]
                | [false, false, false, true, false, false, false, false]
                | [false, false, false, false, true, false, false, false]
                | [false, false, false, false, false, true, false, false]
                | [false, false, false, false, false, false, true, false]
                | [false, false, false, false, false, false, false, true] => true,
                _ => false,
            };
            if is_single_line && self.car_current_location != CarLocation::SingleLine {
                self.car_current_location = CarLocation::SingleLine;
                log::info!("Location updated to: {:?}", self.car_current_location);
            }
        }

        Ok(())
    }

    /// Parking and then exit for number 1 ~ 8.
    pub fn park_and_exit(&mut self, base_speed: f32, num: u8) -> Result<(), CarDriverError> {
        let update_crossing_count = match num {
            1 | 2 => 4,
            3 | 4 => 5,
            7 | 8 => 8,
            5 | 6 => 9,
            _ => unreachable!("Invalid number."),
        };

        let (enter_direction, exit_direction) = match num {
            1 | 3 | 6 | 8 => (TURN_LEFT, TURN_RIGHT),
            2 | 4 | 5 | 7 => (TURN_RIGHT, TURN_LEFT),
            _ => unreachable!("Invalid number."),
        };

        self.go_straight(base_speed, PARKING_TURN_FORWARD_MS)?;
        self.turn_in_place(enter_direction)?;
        self.go_back(-base_speed, 100)?;

        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(update_crossing_count + 1)
        })?;

        self.beep(2)?;
        FreeRtos::delay_ms(PARKING_WAIT_TIME);
        self.beep(3)?;

        self.go_back(-base_speed, BACK_FORWARD_MS)?;

        self.turn(exit_direction)?;

        self.car_current_crossing_count = update_crossing_count;
        self.update_location()?;

        Ok(())
    }

    pub fn car_routing_block(&mut self, base_speed: f32, num: u8) -> Result<(), CarDriverError> {
        FreeRtos::delay_ms(START_END_WAIT_TIME);
        self.beep(3)?;
        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(2)
        })?;
        self.go_straight(base_speed, TURN_FORWARD_MS)?;
        self.turn(TURN_LEFT)?;
        self.go_straight(-base_speed, 15)?;

        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(3)
        })?;
        self.go_straight(base_speed, TURN_FORWARD_MS)?;
        self.turn(TURN_RIGHT)?;

        match num {
            1 | 2 => {
                self.go_straight_until(base_speed, |car| {
                    car.car_current_location == CarLocation::Crossing(4)
                })?;
                self.park_and_exit(base_speed, num)?;
            }
            3 | 4 => {
                self.go_straight_until(base_speed, |car| {
                    car.car_current_location == CarLocation::Crossing(5)
                })?;
                self.park_and_exit(base_speed, num)?;
            }
            _ => {}
        }

        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(6)
        })?;
        self.go_straight(base_speed, TURN_FORWARD_MS)?;
        self.turn(TURN_LEFT + 5.0)?;

        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(7)
        })?;
        self.go_straight(base_speed, TURN_FORWARD_MS)?;
        self.turn(TURN_LEFT)?;

        match num {
            7 | 8 => {
                self.go_straight_until(base_speed, |car| {
                    car.car_current_location == CarLocation::Crossing(8)
                })?;
                self.park_and_exit(base_speed, num)?;
            }

            5 | 6 => {
                self.go_straight_until(base_speed, |car| {
                    car.car_current_location == CarLocation::Crossing(9)
                })?;
                self.park_and_exit(base_speed, num)?;
            }
            _ => {}
        }

        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(10)
        })?;
        self.go_straight(base_speed, TURN_FORWARD_MS + 5)?;
        self.turn(TURN_RIGHT + 10.0)?;
        self.go_straight(-base_speed, 80)?;

        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(11)
        })?;
        self.go_straight(base_speed, TURN_FORWARD_MS)?;
        self.turn(TURN_LEFT)?;

        self.go_straight_until(base_speed, |car| {
            car.car_current_location == CarLocation::Crossing(12)
        })?;

        self.beep(3)?;

        Ok(())
    }
}

fn angle_diff(target: f32, current: f32) -> f32 {
    let mut diff = target - current;
    while diff > 180.0 {
        diff -= 360.0;
    }
    while diff < -180.0 {
        diff += 360.0;
    }
    diff
}
