use crate::error::PhysicsError;

/// Handles simulated time
pub struct SimTime {
    // The frequency of simulation
    hz: u64,
    step_count: u64,
}

impl SimTime {
    pub fn new(hz: u64) -> Result<Self, PhysicsError> {
        if hz > 0 {
            Ok(Self {
                step_count: 0,
                hz,
            })
        } else {
            Err(PhysicsError::InvalidFrequency(hz))
        }
    }

    /// Advance the simulation
    pub fn advance(&mut self) {
        self.step_count += 1;
    }

    /// Get the floating-point `dt` in seconds
    pub fn dt_seconds(&self) -> f64 {
        1.0 / self.hz as f64
    }

    /// Get the total elapsed time in seconds
    pub fn elapsed_seconds(&self) -> f64 {
        self.step_count as f64 / self.hz as f64
    }
}
