use crate::error::PhysicsError;

/// Handles simulated time
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

    pub fn step_count(&self) -> u64 {
        self.step_count
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-12;

    #[test]
    fn zero_hz_is_rejected() {
        let r = SimTime::new(0);
        assert!(matches!(r, Err(PhysicsError::InvalidFrequency(0))));
    }

    #[test]
    fn nonzero_hz_is_accepted() {
        assert!(SimTime::new(1).is_ok());
        assert!(SimTime::new(60).is_ok());
        assert!(SimTime::new(1_000_000).is_ok());
    }

    #[test]
    fn new_time_starts_at_zero() {
        let t = SimTime::new(240).unwrap();
        assert_eq!(t.elapsed_seconds(), 0.0);
    }

    #[test]
    fn dt_is_reciprocal_of_hz() {
        for hz in [1_u64, 30, 60, 240, 1000] {
            let t = SimTime::new(hz).unwrap();
            assert!((t.dt_seconds() - 1.0 / hz as f64).abs() < EPS);
        }
    }

    #[test]
    fn dt_does_not_change_after_advancing() {
        let mut t = SimTime::new(240).unwrap();
        let before = t.dt_seconds();
        for _ in 0..1000 {
            t.advance();
        }
        assert_eq!(t.dt_seconds(), before);
    }

    #[test]
    fn advance_moves_time_by_exactly_one_dt() {
        let mut t = SimTime::new(240).unwrap();
        t.advance();
        assert!((t.elapsed_seconds() - t.dt_seconds()).abs() < EPS);
        t.advance();
        assert!((t.elapsed_seconds() - 2.0 * t.dt_seconds()).abs() < EPS);
    }

    #[test]
    fn hz_steps_make_exactly_one_second() {
        // The "no drift" property: step_count / hz is exact at whole seconds.
        for hz in [1_u64, 30, 60, 120, 240, 1000] {
            let mut t = SimTime::new(hz).unwrap();
            for _ in 0..hz {
                t.advance();
            }
            assert_eq!(t.elapsed_seconds(), 1.0, "hz = {hz}");
        }
    }

    #[test]
    fn no_drift_over_many_steps() {
        // 10 minutes at 240 Hz. Accumulating dt in f64 would drift here;
        // dividing an integer count does not.
        let hz = 240_u64;
        let mut t = SimTime::new(hz).unwrap();
        for _ in 0..(hz * 600) {
            t.advance();
        }
        assert_eq!(t.elapsed_seconds(), 600.0);
    }

    #[test]
    fn non_representable_dt_still_lands_on_whole_seconds() {
        // 1/7 is not exactly representable, but 7 steps at 7 Hz should
        // still give 1.0 because elapsed = count / hz, not count * dt.
        let mut t = SimTime::new(7).unwrap();
        for _ in 0..7 {
            t.advance();
        }
        assert_eq!(t.elapsed_seconds(), 1.0);
    }

    #[test]
    fn elapsed_is_monotonic() {
        let mut t = SimTime::new(60).unwrap();
        let mut prev = t.elapsed_seconds();
        for _ in 0..500 {
            t.advance();
            let now = t.elapsed_seconds();
            assert!(now > prev);
            prev = now;
        }
    }

    #[test]
    fn step_count_matches_number_of_advances() {
        let mut t = SimTime::new(60).unwrap();
        assert_eq!(t.step_count(), 0);
        for _ in 0..123 {
            t.advance();
        }
        assert_eq!(t.step_count(), 123);
    }
}