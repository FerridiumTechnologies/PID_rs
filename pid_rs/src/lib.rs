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


/// Smith-Predictor attempt for processes with dead time (time delays)
pub struct SmithPredictor<const MODEL_ORDER: usize> {
    pid: PidController,
    
    // Process model parameters (for delay-free part)
    model_numerator: [f32; MODEL_ORDER],    // b0, b1, b2, ... 
    model_denominator: [f32; MODEL_ORDER],  // a0, a1, a2, ... (a0 is usually 1.0) 
    
    // Time delay model (in samples)
    delay_samples: usize,
    
    // Model state buffers
    model_input_buffer: [f32; MODEL_ORDER],  // Past inputs u[k-1], u[k-2], ...
    model_output_buffer: [f32; MODEL_ORDER], // Past model outputs y_m[k-1], y_m[k-2], ...
    
    // Delay buffer (FIFO queue for delayed model output)
    delay_buffer: [f32; 256],  // Fixed-size buffer, adjust to your needs nerd!
    delay_write_idx: usize,
    delay_read_idx: usize,
    
    // State variables
    previous_control_output: f32,
    predicted_output: f32,
    model_output: f32,
    
    // Configuration
    model_enabled: bool,
    adaptive_model: bool,
    model_learning_rate: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum SmithPredictorError {
    InvalidDelay,
    InvalidModelParameters,
    BufferTooSmall,
    ModelNotStable,
}

impl<const N: usize> SmithPredictor<N> {
    /// Create a new Smith Predictor with given PID controller and process model
    pub fn new(
        pid: PidController,
        model_numerator: [f32; N],
        model_denominator: [f32; N],
        delay_samples: usize,
    ) -> Result<Self, SmithPredictorError> {
        // Validate model parameters
        if model_denominator[0] == 0.0 {
            return Err(SmithPredictorError::InvalidModelParameters);
        }
        
        // Check for model stability (simplified check)
        let mut sum = 0.0;
        for i in 0..N {
            sum += model_denominator[i].abs(); //computes absolute value of self
        }
        if sum > 10.0 * N as f32 {  // Rough stability check
            return Err(SmithPredictorError::ModelNotStable);
        }
        
        if delay_samples >= 256 {  // Our buffer size
            return Err(SmithPredictorError::BufferTooSmall);
        }
        
        Ok(Self {
            pid,
            model_numerator,
            model_denominator,
            delay_samples,
            model_input_buffer: [0.0; N],
            model_output_buffer: [0.0; N],
            delay_buffer: [0.0; 256],
            delay_write_idx: 0,
            delay_read_idx: delay_samples, // Read lags write by delay_samples
            previous_control_output: 0.0,
            predicted_output: 0.0,
            model_output: 0.0,
            model_enabled: true,
            adaptive_model: false,
            model_learning_rate: 0.001,
        })
    }
    
    /// Compute control output using Smith Predictor architecture, with setpoint & measurement
    pub fn compute(&mut self, setpoint: f32, measurement: f32) -> f32 {
        // 1. Compute the delay-free model output
        self.model_output = self.compute_model_output(self.previous_control_output);
        
        // 2. Compute delayed model output (from buffer)
        let delayed_model_output = self.get_delayed_model_output();
        
        // 3. Update delay buffer with current model output
        self.update_delay_buffer(self.model_output);
        
        // 4. Compute model error (difference between real process and delayed model)
        let model_error = measurement - delayed_model_output;
        
        // 5. Compute predicted present output (compensated for the delay)
        self.predicted_output = self.model_output + model_error;
        
        // 6. Use PID to compute control output based on predicted output
        let control_output = if self.model_enabled {
            // Smith Predictor mode: PID sees predicted present output
            self.pid.compute(setpoint, self.predicted_output)
        } else {
            // Fallback to regular PID (for testing or model failure)
            self.pid.compute(setpoint, measurement)
        };
        
        // 7. Store control output for next iteration
        self.previous_control_output = control_output;
        
        // 8. Optional: adaptive model adjustment
        if self.adaptive_model {
            self.adapt_model(measurement, delayed_model_output, control_output);
        }
        
        control_output
    }
    
    /// Asynchronous version with proper timing
    pub async fn compute_async(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let now = Instant::now();
        
        // Use the PID's timing logic
        if let Some(last_time) = self.pid.last_time {
            let elapsed = now.duration_since(last_time);
            if elapsed < self.pid.sample_time {
                Timer::after(self.pid.sample_time - elapsed).await;
            }
        }
        
        self.compute(setpoint, measurement)
    }
    
    /// Compute the delay-free model output using difference equation
    fn compute_model_output(&mut self, input: f32) -> f32 {
        if !self.model_enabled {
            return 0.0;
        }
        
        // Shift input buffer
        for i in (1..N).rev() { //reverse the iterator's direction
            self.model_input_buffer[i] = self.model_input_buffer[i - 1];
        }
        self.model_input_buffer[0] = input;
        
        // Compute output using difference equation: TODO: Improve this later with less confusing arithmetic
        //y[k] = (b0*u[k] + b1*u[k-1] + ...) - (a1*y[k-1] + a2*y[k-2] + ...)

        let mut output = 0.0; //simple
        
        // Numerator terms (b coefficients with input)
        for i in 0..N {
            output += self.model_numerator[i] * self.model_input_buffer[i];
        }
        
        // Denominator terms (a coefficients with past outputs, a0 is assumed 1)
        for i in 1..N {
            output -= self.model_denominator[i] * self.model_output_buffer[i - 1];
        }
        
        // Normalize by a0 (which should be 1.0 for proper form)
        if self.model_denominator[0] != 0.0 {
            output /= self.model_denominator[0];
        }
        
        // Shift output buffer
        for i in (1..N).rev() {
            self.model_output_buffer[i] = self.model_output_buffer[i - 1];
        }
        self.model_output_buffer[0] = output;
        
        output
    }
    
    /// Get the delayed model output from buffer
    fn get_delayed_model_output(&self) -> f32 {
        if !self.model_enabled || self.delay_samples == 0 {
            return self.model_output;
        }
        
        self.delay_buffer[self.delay_read_idx]
    }
    
    /// Update the delay buffer with new model output
    fn update_delay_buffer(&mut self, new_output: f32) {
        if self.delay_samples == 0 {
            return;
        }
        
        // Write new value
        self.delay_buffer[self.delay_write_idx] = new_output;
        
        // Update indices with wrap-around
        self.delay_write_idx = (self.delay_write_idx + 1) % 256;
        self.delay_read_idx = (self.delay_read_idx + 1) % 256;
    }
    
    /// Simple adaptive model adjustment attempt (LMS-like algorithm). Haha!
    fn adapt_model(&mut self, actual_output: f32, predicted_output: f32, _control_input: f32) {
        let error = actual_output - predicted_output;
        
        // Adjust model parameters (simplified gradient descent)
        // This is a simplified version - a real implementation would need more sophistication (<_>)
        for i in 0..N {
            if i < self.model_input_buffer.len() {
                // Adjust numerator coefficients based on input contribution
                let gradient = self.model_input_buffer[i] * error;
                self.model_numerator[i] += self.model_learning_rate * gradient;
            }
            
            if i > 0 && i - 1 < self.model_output_buffer.len() {
                // Adjust denominator coefficients (except a0)
                let gradient = self.model_output_buffer[i - 1] * error;
                self.model_denominator[i] += self.model_learning_rate * gradient;
            }
        }
        
        // Ensure a0 stays at 1.0 (standard form)
        if self.model_denominator[0] != 1.0 {
            // Normalize all coefficients
            let a0 = self.model_denominator[0];
            for coeff in self.model_denominator.iter_mut() { //return the mutable iterator to modify each value
                *coeff /= a0;
            }
            for coeff in self.model_numerator.iter_mut() {
                *coeff /= a0; //divides the left operand by the right operand and assigns the result back to the left operand
                //just like x = x / y
            }
            self.model_denominator[0] = 1.0;
        }
    }
    
    /// Enable/disable Smith Predictor (fallback to regular PID)
    pub fn enable_model(&mut self, enabled: bool) {
        self.model_enabled = enabled;
        
        if enabled {
            // Reset model state when enabling
            self.model_input_buffer = [0.0; N];
            self.model_output_buffer = [0.0; N];
            self.delay_buffer = [0.0; 256];
            self.delay_write_idx = 0;
            self.delay_read_idx = self.delay_samples;
            self.previous_control_output = 0.0;
        }
    }
    
    /// Enable/disable adaptive model adjustment because my adptation algorithm is too basic
    pub fn enable_adaptation(&mut self, enabled: bool) {
        self.adaptive_model = enabled;
    }
    
    /// Set model learning rate for adaptation
    pub fn set_learning_rate(&mut self, rate: f32) -> Result<(), SmithPredictorError> {
        if rate <= 0.0 || rate > 1.0 || rate.is_nan() || rate.is_infinite() {
            return Err(SmithPredictorError::InvalidModelParameters);
        }
        self.model_learning_rate = rate;
        Ok(())
    }
    
    /// Get current model parameters
    pub fn get_model_parameters(&self) -> ([f32; N], [f32; N]) {
        (self.model_numerator, self.model_denominator)
    }
    
    /// Get predicted output (for monitoring/debugging)
    pub fn get_predicted_output(&self) -> f32 {
        self.predicted_output
    }
    
    /// Get model output (for monitoring/debugging)
    pub fn get_model_output(&self) -> f32 {
        self.model_output
    }
    
    /// Reset the Smith Predictor state
    pub fn reset(&mut self) {
        self.pid.reset();
        self.model_input_buffer = [0.0; N];
        self.model_output_buffer = [0.0; N];
        self.delay_buffer = [0.0; 256];
        self.delay_write_idx = 0;
        self.delay_read_idx = self.delay_samples;
        self.previous_control_output = 0.0;
        self.predicted_output = 0.0;
        self.model_output = 0.0;
    }
    
    /// Get reference to internal PID controller for tuning
    pub fn pid_mut(&mut self) -> &mut PidController {
        &mut self.pid
    }
    
    /// Get immutable reference to internal PID controller. Might be useful
    pub fn pid(&self) -> &PidController {
        &self.pid
    }
}

/// Helper functions for creating common process models
pub mod smith_models {
    //use super::*;
    
    /// Create a first-order plus time delay (FOPTD) model
    /// K: process gain, tau: time constant, theta: time delay (in samples)
    pub fn create_foptd_model<const N: usize>(
        k: f32, 
        tau: f32, 
        _theta_samples: usize
    ) -> ([f32; N], [f32; N]) {
        // First-order discrete model: y[k] = a*y[k-1] + b*u[k-1]
        // where a = exp(-dt/tau), b = K*(1 - a)
        // Using N=2 for first-order model
        
        assert!(N >= 2, "First-order model requires N >= 2");
        
        let dt = 1.0; // Assuming unit sampling time for normalization
    let a = libm::expf(-dt / tau);
        let b = k * (1.0 - a);
        
        let mut numerator = [0.0; N];
        let mut denominator = [0.0; N];
        
        // b0 = 0, b1 = b (one sample delay)
        numerator[1] = b;
        
        // a0 = 1, a1 = -a
        denominator[0] = 1.0;
        denominator[1] = -a;
        
        (numerator, denominator)
    }
    
    /// Create a second-order plus time delay (SOPTD) model...needs some improvement
    pub fn create_soptd_model<const N: usize>(
        k: f32,
        tau1: f32,
        tau2: f32,
        _theta_samples: usize
    ) -> ([f32; N], [f32; N]) {
        // Second-order discrete model
        assert!(N >= 3, "Second-order model requires N >= 3");
        
        let dt = 1.0;
        let a1 = libm::expf(-dt / tau1);
        let a2 = libm::expf(-dt / tau2);
        
        // Simplified coefficients - TODO: improve to use proper discretization
        let mut numerator = [0.0; N];
        let mut denominator = [0.0; N];
        
        numerator[2] = k * (1.0 - a1) * (1.0 - a2);
        denominator[0] = 1.0;
        denominator[1] = -(a1 + a2);
        denominator[2] = a1 * a2;
        
        (numerator, denominator)
    }
    
    /// Create an integrator plus time delay model
    pub fn create_integrator_model<const N: usize>(
        k: f32,
        _theta_samples: usize
    ) -> ([f32; N], [f32; N]) {
        // Integrator: y[k] = y[k-1] + K*u[k-1]*dt
        assert!(N >= 2, "Integrator model requires N >= 2");
        
        let dt = 1.0;
        let mut numerator = [0.0; N];
        let mut denominator = [0.0; N];
        
        numerator[1] = k * dt;
        denominator[0] = 1.0;
        denominator[1] = -1.0; // y[k-1] coefficient
        
        (numerator, denominator)
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

///Smith-Predictor tests
#[cfg(test)]
mod smith_tests {
    use super::*;
    use embassy_time::Duration;
    
    #[test]
    fn test_smith_predictor_creation() {
        // Create a PID controller
        let pid = PidController::new(1.0, 0.1, 0.05)
            .unwrap()
            .with_output_limits(-10.0, 10.0)
            .unwrap()
            .with_sample_time(Duration::from_millis(10))
            .unwrap();
        
        // Create a first-order model (K=1.0, tau=2.0)
        let (num, den) = smith_models::create_foptd_model::<2>(1.0, 2.0, 5);
        
        // Create Smith Predictor with 5 samples delay
        let smith = SmithPredictor::new(pid, num, den, 5);
        
        assert!(smith.is_ok());
    }
    
    #[test]
    fn test_smith_predictor_computation() {
        let pid = PidController::new(2.0, 0.2, 0.1)
            .unwrap()
            .with_output_limits(-5.0, 5.0)
            .unwrap();
        
        // Simple gain model: y[k] = 0.5*u[k-1]
        let numerator = [0.0, 0.5];  // b0=0, b1=0.5
        let denominator = [1.0, 0.0]; // a0=1, a1=0
        
        let mut smith = SmithPredictor::new(pid, numerator, denominator, 3)
            .unwrap();
        
        // Test computation
        let output = smith.compute(100.0, 0.0);
        
        // First computation should produce some output
        assert!(output.abs() > 0.0);
        
        // Model should be enabled by default
        assert!(smith.model_enabled);
    }
    
    #[test]
    fn test_model_enable_disable() {
        let pid = PidController::new(1.0, 0.1, 0.01).unwrap();
        let (num, den) = smith_models::create_foptd_model::<2>(1.0, 1.0, 2);
        
        let mut smith = SmithPredictor::new(pid, num, den, 2).unwrap();
        
        // Initially enabled
        assert!(smith.model_enabled);
        
        // Disable - should fall back to regular PID
        smith.enable_model(false);
        assert!(!smith.model_enabled);
        
        // Re-enable
        smith.enable_model(true);
        assert!(smith.model_enabled);
    }
    
    #[test]
    fn test_smith_predictor_reset() {
        let pid = PidController::new(1.0, 0.1, 0.01).unwrap();
        let (num, den) = smith_models::create_foptd_model::<2>(1.0, 1.0, 2);
        
        let mut smith = SmithPredictor::new(pid, num, den, 2).unwrap();
        
        // Do some computations
        smith.compute(50.0, 0.0);
        smith.compute(50.0, 10.0);
        
        // Reset
        smith.reset();
        
        // After reset, internal buffers should be cleared
        // (This is mostly to ensure no panic on reset)
        let output = smith.compute(50.0, 0.0);
        assert!(output.is_finite());
    }
}

//TODO: Do more sensible tests!
