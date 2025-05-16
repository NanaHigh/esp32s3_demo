/// Speed PID controller structure.
#[derive(Debug)]
pub struct SpeedPidController {
    kp: f32,         // Proportional coefficient
    ki: f32,         // Integral coefficient
    kd: f32,         // Derivative coefficient
    target: f32,     // Target speed
    error_sum: f32,  // Accumulated error
    last_error: f32, // Previous error
    output: f32,     // PWM output value (adjustment)
    last_d: f32,     // Last derivative term
    dead_zone: f32,  // Dead zone range
}

impl SpeedPidController {
    /// Create a new PID controller.
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            target: 0.0,
            error_sum: 0.0,
            last_error: 0.0,
            output: 0.0,
            last_d: 0.0,
            dead_zone: 0.08, // Set dead zone range
        }
    }

    /// Calculate PID output.
    pub fn calculate(&mut self, current: f32) -> f32 {
        let error = self.target - current;

        // Add dead zone control
        if error.abs() < self.dead_zone {
            return self.output;
        }

        // Calculate PID components
        let p = self.kp * error;

        // Limit integral range
        self.error_sum = (self.error_sum + error).clamp(-30.0, 30.0);
        let i = self.ki * self.error_sum;

        // Derivative term filtering
        let d_raw = self.kd * (error - self.last_error);
        let d = 0.8 * self.last_d + 0.2 * d_raw; // Low-pass filtering
        self.last_d = d;

        self.last_error = error;

        // Limit single adjustment amount
        let adjustment = (p + i + d).clamp(-3.0, 3.0);
        self.output = (self.output + adjustment).clamp(-100.0, 100.0);

        self.output
    }

    /// Set PID target.
    pub fn set_target(&mut self, target: f32) {
        self.target = target;
        self.error_sum = 0.0;
        self.last_error = 0.0;
        self.output = 0.0;
    }
}

/// Position PID controller structure.
#[derive(Debug)]
pub struct PositionPidController {
    kp: f32,         // Proportional coefficient
    ki: f32,         // Integral coefficient
    kd: f32,         // Derivative coefficient
    target: f32,     // Target angle
    error_sum: f32,  // Accumulated error
    last_error: f32, // Previous error
    output: f32,     // Output value (angle adjustment)
    dead_zone: f32,  // Dead zone range
}

impl PositionPidController {
    /// Create a new position PID controller.
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            target: 0.0,
            error_sum: 0.0,
            last_error: 0.0,
            output: 0.0,
            dead_zone: 0.8, // Set dead zone for angle control
        }
    }

    /// Calculate PID output.
    pub fn calculate(&mut self, current: f32) -> f32 {
        let error = self.target - current;

        // Add dead zone control
        if error.abs() < self.dead_zone {
            return 0.0;
        }

        // Calculate PID components
        let p = self.kp * error;

        // Limit integral range
        self.error_sum = (self.error_sum + error).clamp(-45.0, 45.0);
        let i = self.ki * self.error_sum;

        let d = self.kd * (error - self.last_error);
        self.last_error = error;

        // Limit output angle range
        self.output = (p + i + d).clamp(-90.0, 90.0);

        self.output
    }

    /// Set PID target angle.
    pub fn set_target(&mut self, target: f32) {
        self.target = target;
        self.error_sum = 0.0;
        self.last_error = 0.0;
        self.output = 0.0;
    }
}
