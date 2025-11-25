#![no_std]

use embassy_time::{Instant, Timer, Duration};
//use num_traits::float::FloatCore;


/*#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}*/

#[derive(Debug, Clone, Copy)]
pub struct PidController {
    //gains
    kp: f32,
    ki: f32, 
    kd: f32,
    
    //Setpoint weighting factors (0.0 to 1.0)
    setpoint_weight_proportional: f32,    // b
    setpoint_weight_derivative: f32,      // c
    
    //State vars
    integral: f32,
    previous_error: f32,
    previous_measurement: f32,
    previous_setpoint: f32,
    
    //Limits
    output_limits: (f32, f32),
    integral_limits: (f32, f32),
    
    //Timing
    last_time: Option<Instant>,
    sample_time: Duration,
    
    //Config
    proportional_on_measurement: bool,
    derivative_on_measurement: bool,
    
    //Filtering
    derivative_filter_alpha: f32,
    filtered_derivative: f32,
    
    //Anti-windup
    clamping_anti_windup: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum PidError {
    InvalidGains,
    InvalidLimits,
    InvalidSampleTime,
    InvalidWeightingFactor,
}

impl PidController {
    ///Create a new PID controller with the specified gains
    pub fn new(kp: f32, ki: f32, kd: f32) -> Result<Self, PidError> {
        if kp < 0.0 || ki < 0.0 || kd < 0.0 || 
           kp.is_nan() || ki.is_nan() || kd.is_nan() ||
           kp.is_infinite() || ki.is_infinite() || kd.is_infinite() {
            return Err(PidError::InvalidGains);
        }
        
        Ok(Self {
            kp,
            ki,
            kd,
            setpoint_weight_proportional: 1.0, // Default: traditional PID
            setpoint_weight_derivative: 1.0,   // Default: traditional PID
            integral: 0.0,
            previous_error: 0.0,
            previous_measurement: 0.0,
            previous_setpoint: 0.0,
            output_limits: (f32::MIN, f32::MAX),
            integral_limits: (f32::MIN, f32::MAX),
            last_time: None,
            sample_time: Duration::from_millis(100),
            proportional_on_measurement: false,
            derivative_on_measurement: true,
            derivative_filter_alpha: 0.1,
            filtered_derivative: 0.0,
            clamping_anti_windup: true,
        })
    }
    
    ///Sets proportional setpoint weighting factor b
    ///b = 0.0: proportional term depends only on measurement (no setpoint kick)
    ///b = 1.0: traditional PID with full setpoint in proportional term
    pub fn with_setpoint_weight_proportional(mut self, b: f32) -> Result<Self, PidError> {
        if !(0.0..=1.0).contains(&b) || b.is_nan() {
            return Err(PidError::InvalidWeightingFactor);
        }
        self.setpoint_weight_proportional = b;
        Ok(self)
    }
    
    ///Sets derivative setpoint weighting factor c
    ///c = 0.0: derivative term depends only on measurement (no derivative kick)
    ///c = 1.0: traditional PID with full setpoint in derivative term
    pub fn with_setpoint_weight_derivative(mut self, c: f32) -> Result<Self, PidError> {
        if !(0.0..=1.0).contains(&c) || c.is_nan() {
            return Err(PidError::InvalidWeightingFactor);
        }
        self.setpoint_weight_derivative = c;
        Ok(self)
    }
    
    ///Sets both setpoint weighting factors
    pub fn with_setpoint_weighting(mut self, b: f32, c: f32) -> Result<Self, PidError> {
        if !(0.0..=1.0).contains(&b) || !(0.0..=1.0).contains(&c) || 
           b.is_nan() || c.is_nan() {
            return Err(PidError::InvalidWeightingFactor);
        }
        self.setpoint_weight_proportional = b;
        self.setpoint_weight_derivative = c;
        Ok(self)
    }
    
    ///Sets output limits to prevent actuator saturation
    pub fn with_output_limits(mut self, min: f32, max: f32) -> Result<Self, PidError> {
        if min >= max || min.is_nan() || max.is_nan() || min.is_infinite() || max.is_infinite() {
            return Err(PidError::InvalidLimits);
        }
        self.output_limits = (min, max);
        Ok(self)
    }
    
    ///Sets integral limits to prevent integral windup
    pub fn with_integral_limits(mut self, min: f32, max: f32) -> Result<Self, PidError> {
        if min >= max || min.is_nan() || max.is_nan() || min.is_infinite() || max.is_infinite() {
            return Err(PidError::InvalidLimits);
        }
        self.integral_limits = (min, max);
        Ok(self)
    }
    
    ///Sets the sample time for discrete control
    pub fn with_sample_time(mut self, sample_time: Duration) -> Result<Self, PidError> {
        if sample_time.as_millis() == 0 {
            return Err(PidError::InvalidSampleTime);
        }
        self.sample_time = sample_time;
        Ok(self)
    }
    
    ///Enables/disables proportional-on-measurement
    pub fn with_proportional_on_measurement(mut self, enabled: bool) -> Self {
        self.proportional_on_measurement = enabled;
        self
    }
    
    ///Enables/disables derivative-on-measurement
    pub fn with_derivative_on_measurement(mut self, enabled: bool) -> Self {
        self.derivative_on_measurement = enabled;
        self
    }
    
    ///Sets derivative filter time constant
    pub fn with_derivative_filter(mut self, time_constant: f32) -> Self {
        let dt = self.sample_time.as_secs() as f32;
        self.derivative_filter_alpha = dt / (time_constant + dt); // Simple first-order filter
        self
    }
    
    ///Enables/disables clamping anti-windup
    pub fn with_anti_windup(mut self, enabled: bool) -> Self {
        self.clamping_anti_windup = enabled;
        self
    }
    
    /// Resets the controller state
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.previous_measurement = 0.0;
        self.previous_setpoint = 0.0;
        self.filtered_derivative = 0.0;
        self.last_time = None;
    }
    
    ///Computes the control output asynchronously with proper timing
    pub async fn compute_async(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let now = Instant::now();
        
        // Wait for sample time if needed
        if let Some(last_time) = self.last_time {
            let elapsed = now.duration_since(last_time);
            if elapsed < self.sample_time {
                Timer::after(self.sample_time - elapsed).await;
            }
        }
        
        self.compute_internal(setpoint, measurement, Instant::now())
    }
    /// Computes the control output synchronously
    pub fn compute(&mut self, setpoint: f32, measurement: f32) -> f32 {
        self.compute_internal(setpoint, measurement, Instant::now())
    }
    
    /// Internal computation logic with setpoint weighting
    fn compute_internal(&mut self, setpoint: f32, measurement: f32, now: Instant) -> f32 {
        let dt = self.calculate_dt(now);
        
        // Return previous output if not enough time has passed
        if dt < self.sample_time.as_secs() as f32 {
            return self.clamp_output(self.previous_output());
        }
        
        let error = setpoint - measurement; // Current error
        
        //Proportional term with setpoint weighting
        let proportional = if self.proportional_on_measurement {
            //P-on-M: proportional acts only on measurement
            -self.kp * measurement
        } else {
            //Standard P with setpoint weighting: Kp * (b*setpoint - measurement)
            self.kp * (self.setpoint_weight_proportional * setpoint - measurement)
        };
        
        //Integral term (always uses full error for integral action)
        self.integral += self.ki * error * dt;
        self.integral = self.clamp_integral(self.integral);
        
        //Derivative term with setpoint weighting
        let derivative = if self.derivative_on_measurement {
            //D-on-M: derivative acts only on measurement
            -self.kd * self.calculate_derivative(measurement, dt)
        } else {
            //Standard D with setpoint weighting
            let weighted_setpoint_change = if dt > 0.0 {
                self.setpoint_weight_derivative * (setpoint - self.previous_setpoint) / dt
            } else {
                0.0
            };
            let measurement_derivative = self.calculate_derivative(measurement, dt);
            
            self.kd * (weighted_setpoint_change - measurement_derivative)
        };
        
        // Compute output
        let output = proportional + self.integral + derivative; // Raw output before clamping
        let clamped_output = self.clamp_output(output);
        
        // Anti-windup: stop integrating if output is saturated
        if self.clamping_anti_windup {
            self.apply_anti_windup(output, clamped_output, setpoint, measurement);
        }
        
        // Update state
        self.previous_error = error;
        self.previous_measurement = measurement;
        self.previous_setpoint = setpoint;
        self.last_time = Some(now);
        
        clamped_output
    }
    
    /// Gets the current controller gains (kp, ki, kd)
    pub fn gains(&self) -> (f32, f32, f32) {
        (self.kp, self.ki, self.kd)
    }
    
    /// Gets the current setpoint weighting factors
    pub fn setpoint_weights(&self) -> (f32, f32) {
        (self.setpoint_weight_proportional, self.setpoint_weight_derivative)
    }
    
    /// Updates controller gains (bumpless transfer)
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) -> Result<(), PidError> {
        if kp < 0.0 || ki < 0.0 || kd < 0.0 || 
           kp.is_nan() || ki.is_nan() || kd.is_nan() ||
           kp.is_infinite() || ki.is_infinite() || kd.is_infinite() {
            return Err(PidError::InvalidGains);
        }
        
        // Bumpless transfer: adjust integral to maintain same output
        if self.kp != 0.0 {
            self.integral = self.integral * self.kp / kp;
        }
        
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        
        Ok(())
    }
    
    /// Updates setpoint weighting factors
    pub fn set_setpoint_weights(&mut self, b: f32, c: f32) -> Result<(), PidError> {
        if !(0.0..=1.0).contains(&b) || !(0.0..=1.0).contains(&c) || 
           b.is_nan() || c.is_nan() {
            return Err(PidError::InvalidWeightingFactor);
        }
        
        self.setpoint_weight_proportional = b;
        self.setpoint_weight_derivative = c;
        
        Ok(())
    }
    
    /// Returns the current integral value
    pub fn integral(&self) -> f32 {
        self.integral
    }
    
    /// Returns true if the controller is initialized (has previous state)
    pub fn is_initialized(&self) -> bool {
        self.last_time.is_some()
    }
    
    // Private helper methods
    fn calculate_dt(&mut self, now: Instant) -> f32 {
        match self.last_time {
            Some(last_time) => now.duration_since(last_time).as_secs() as f32,
            None => {
                self.last_time = Some(now);
                self.sample_time.as_secs() as f32
            }
        }
    }
    fn calculate_derivative(&mut self, value: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }
        
        let derivative = (value - self.previous_measurement) / dt;
        
        // Apply "low-pass filter" to derivative ?
        self.filtered_derivative = self.derivative_filter_alpha * derivative + 
            (1.0 - self.derivative_filter_alpha) * self.filtered_derivative;
        
        self.filtered_derivative
    }
    
    fn clamp_output(&self, output: f32) -> f32 {
        output.clamp(self.output_limits.0, self.output_limits.1)
    }
    
    fn clamp_integral(&self, integral: f32) -> f32 {
        integral.clamp(self.integral_limits.0, self.integral_limits.1)
    }
    
    // Anti-windup adjustment
    fn apply_anti_windup(&mut self, raw_output: f32, clamped_output: f32, setpoint: f32, measurement: f32) {
        if (raw_output - clamped_output).abs() > f32::EPSILON {
            // Output is saturated, adjust integral to prevent windup
            let proportional = if self.proportional_on_measurement {
                -self.kp * measurement
            } else {
                self.kp * (self.setpoint_weight_proportional * setpoint - measurement)
            };
            
            let derivative = if self.derivative_on_measurement {
                -self.kd * self.filtered_derivative
            } else {
                let dt = self.sample_time.as_secs() as f32;
                let weighted_setpoint_change = if dt > 0.0 {
                    self.setpoint_weight_derivative * (setpoint - self.previous_setpoint) / dt
                } else {
                    0.0
                };
                self.kd * (weighted_setpoint_change - self.filtered_derivative)
            };
            
            self.integral = clamped_output - proportional - derivative;
            self.integral = self.clamp_integral(self.integral);
        }
    }
    
    fn previous_output(&self) -> f32 {
        let proportional = if self.proportional_on_measurement {
            -self.kp * self.previous_measurement
        } else {
            self.kp * (self.setpoint_weight_proportional * self.previous_setpoint - self.previous_measurement)
        };
        
        let derivative = if self.derivative_on_measurement {
            -self.kd * self.filtered_derivative
        } else {
            self.kd * self.filtered_derivative // Simplified for previous output
        };
        
        proportional + self.integral + derivative
    }
}

/// Some example controllers to demonstrate usage of setpoint weighting
pub mod setpoint_weighting_examples {
    use super::*;
    //use defmt::*;
    
    /// Example showing how setpoint weighting eliminates setpoint kick
    pub struct SmoothPositionController {
        pid: PidController,
    }
    
    impl SmoothPositionController {
        pub fn new() -> Result<Self, PidError> {
            // Use setpoint weighting to eliminate derivative kick on setpoint changes
            let pid = PidController::new(1.5, 0.05, 0.2)?
                .with_output_limits(-10.0, 10.0)?
                .with_setpoint_weighting(1.0, 0.0)?  // No derivative kick (c=0)
                .with_sample_time(Duration::from_millis(10))?;
            
            Ok(Self { pid })
        }
        
        pub fn compute(&mut self, target_position: f32, current_position: f32) -> f32 {
            self.pid.compute(target_position, current_position)
        }
    }
    
    /// Example for processes where setpoint changes should be very gentle
    pub struct GentleSetpointController {
        pid: PidController,
    }
    
    impl GentleSetpointController {
        pub fn new() -> Result<Self, PidError> {
            // Very gentle setpoint following - proportional and derivative act mainly on measurement
            let pid = PidController::new(2.0, 0.1, 0.3)?
                .with_output_limits(0.0, 100.0)?
                .with_setpoint_weighting(0.3, 0.1)?  // Very gentle setpoint response
                .with_sample_time(Duration::from_millis(50))?;
            
            Ok(Self { pid })
        }
    }
    
    /// Traditional PID for comparison (full setpoint kick)
    pub struct TraditionalPidController {
        pid: PidController,
    }
    
    impl TraditionalPidController {
        pub fn new() -> Result<Self, PidError> {
            // Traditional PID with full setpoint weighting (b=1, c=1)
            let pid = PidController::new(1.5, 0.05, 0.2)?
                .with_output_limits(-10.0, 10.0)?
                .with_setpoint_weighting(1.0, 1.0)?  // Traditional behavior
                .with_sample_time(Duration::from_millis(10))?;
            
            Ok(Self { pid })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_setpoint_weighting_creation() {
        let pid = PidController::new(1.0, 0.1, 0.01)
            .unwrap()
            .with_setpoint_weighting(0.5, 0.3)
            .unwrap();
        
        assert_eq!(pid.setpoint_weights(), (0.5, 0.3));
    }
    
    #[test]
    fn test_invalid_setpoint_weights() {
        assert!(PidController::new(1.0, 0.1, 0.01)
            .unwrap()
            .with_setpoint_weighting(1.5, 0.5).is_err()); // b > 1.0
        
        assert!(PidController::new(1.0, 0.1, 0.01)
            .unwrap()
            .with_setpoint_weighting(0.5, -0.1).is_err()); // c < 0.0
    }
    
    #[test]
    fn test_no_derivative_kick() {
        let mut pid = PidController::new(1.0, 0.0, 1.0)
            .unwrap()
            .with_setpoint_weighting(1.0, 0.0) // No derivative kick
            .unwrap()
            .with_sample_time(Duration::from_millis(10))
            .unwrap();
        
        // First computation
        let _output1 = pid.compute(100.0, 0.0);
        
        // Large setpoint change - with c=0, derivative shouldn't cause large output spike
        let output2 = pid.compute(200.0, 50.0);
        
        // Output should be reasonable, not extremely large due to derivative kick
        assert!(output2.abs() < 500.0);
    }
    
    #[test]
    fn test_gentle_proportional_response() {
        let mut pid = PidController::new(2.0, 0.0, 0.0)
            .unwrap()
            .with_setpoint_weighting(0.2, 1.0) // Gentle proportional response
            .unwrap();
        
        let output = pid.compute(100.0, 0.0);
        
        // With b=0.2, proportional term is only 0.2 * 2.0 * 100 = 40
        // Instead of full 2.0 * 100 = 200
        assert!(output < 50.0);
    }
}
