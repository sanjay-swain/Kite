use crate::{
    dynamics::{Dynamics, sources::ForceSource},
    system::{
        loads::Load,
        properties::{BodyId, Properties},
        state::{State, StateDerivative},
    },
};

pub struct NewtonEuler {
    loads: Vec<Load>,
    force_sources: Vec<Box<dyn ForceSource>>,
}

impl NewtonEuler {
    pub fn new(n: usize) -> Self {
        Self {
            loads: vec![Load::default(); n],
            force_sources: Vec::new(),
        }
    }

    fn reset(&mut self) {
        for load in &mut self.loads {
            load.reset();
        }
    }
}

impl Dynamics for NewtonEuler {
    fn set_force_sources(&mut self, sources: Vec<Box<dyn ForceSource>>) {
        self.force_sources = sources;
    }

    fn derivative(
        &mut self,
        properties: &Properties,
        states: &[State],
        t: f64,
        state_derivatives: &mut [StateDerivative],
    ) {
        // Clear out the old loads.
        self.reset();

        // First based on current state find the forces from the required sources.
        for force in &self.force_sources {
            force.compute_loads(&properties, &states, t, &mut self.loads);
        }

        //TODO: Add Caller inputs here

        //TODO: Add constraint solver loads here

        for i in 0..self.loads.len() {
            state_derivatives[i].velocity = states[i].velocity;
            state_derivatives[i].acceleration =
                self.loads[i].force() * properties.mass_inverse(BodyId::new(i));
            state_derivatives[i].angular_velocity = states[i].angular_velocity;
            state_derivatives[i].angular_acceleration = properties.inertia_inverse(BodyId::new(i))
                * (self.loads[i].torque()
                    - states[i]
                        .angular_velocity
                        .cross(properties.inertia(BodyId::new(i)) * states[i].angular_velocity));
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DMat3, DQuat, DVec3};
    use std::cell::Cell;
    use std::rc::Rc;
    use crate::dynamics::sources::gravity_source::ConstantGravity;
    use crate::system::loads::{Force, Frame};

    const EPS: f64 = 1e-9;

    fn rest_state() -> State {
        State {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        }
    }

    fn props(masses: &[f64]) -> Properties {
        Properties::new(masses.to_vec(), vec![DMat3::IDENTITY; masses.len()]).unwrap()
    }

    fn zero_derivs(n: usize) -> Vec<StateDerivative> {
        vec![StateDerivative::ZERO; n]
    }

    /// Test source: adds a fixed world-frame force to every body.
    struct PushAll(DVec3);
    impl ForceSource for PushAll {
        fn compute_loads(&self, _p: &Properties, states: &[State], _t: f64, loads: &mut [Load]) {
            for (load, s) in loads.iter_mut().zip(states) {
                load.add_force(Force::new(self.0, DVec3::ZERO, Frame::World), s.orientation);
            }
        }
    }

    /// Test source: counts how many times it is evaluated.
    struct Counter(Rc<Cell<usize>>);
    impl ForceSource for Counter {
        fn compute_loads(&self, _p: &Properties, _s: &[State], _t: f64, _l: &mut [Load]) {
            self.0.set(self.0.get() + 1);
        }
    }

    /// Runs one derivative evaluation on a single body of the given mass.
    fn accel_of(d: &mut NewtonEuler, mass: f64) -> DVec3 {
        let mut out = zero_derivs(1);
        d.derivative(&props(&[mass]), &[rest_state()], 0.0, &mut out);
        out[0].acceleration
    }

    // ---------- registration ----------

    #[test]
    fn new_dynamics_has_no_sources() {
        let d = NewtonEuler::new(3);
        assert_eq!(d.force_sources.len(), 0);
    }

    #[test]
    fn setting_sources_stores_them() {
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![
            Box::new(PushAll(DVec3::X)),
            Box::new(PushAll(DVec3::Y)),
        ]);
        assert_eq!(d.force_sources.len(), 2);
    }

    #[test]
    fn setting_sources_again_replaces_the_old_ones() {
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![Box::new(PushAll(DVec3::X))]);
        d.set_force_sources(vec![Box::new(PushAll(DVec3::Y))]);
        assert_eq!(d.force_sources.len(), 1);
        // Only the second source is active: a = (0,1,0), not (1,1,0).
        assert!(accel_of(&mut d, 1.0).abs_diff_eq(DVec3::Y, EPS));
    }

    #[test]
    fn setting_empty_list_clears_sources() {
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![Box::new(PushAll(DVec3::X))]);
        d.set_force_sources(vec![]);
        assert_eq!(d.force_sources.len(), 0);
        assert!(accel_of(&mut d, 1.0).abs_diff_eq(DVec3::ZERO, EPS));
    }

    // ---------- force to acceleration ----------

    #[test]
    fn no_sources_gives_zero_acceleration() {
        let mut d = NewtonEuler::new(1);
        assert!(accel_of(&mut d, 2.0).abs_diff_eq(DVec3::ZERO, EPS));
    }

    #[test]
    fn registered_source_affects_derivative() {
        // F = (6,0,0), m = 2  =>  a = (3,0,0)
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![Box::new(PushAll(DVec3::new(6.0, 0.0, 0.0)))]);
        assert!(accel_of(&mut d, 2.0).abs_diff_eq(DVec3::new(3.0, 0.0, 0.0), EPS));
    }

    #[test]
    fn multiple_sources_sum() {
        // (1,0,0) + (0,2,0) on m = 1  =>  a = (1,2,0)
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![
            Box::new(PushAll(DVec3::X)),
            Box::new(PushAll(DVec3::new(0.0, 2.0, 0.0))),
        ]);
        assert!(accel_of(&mut d, 1.0).abs_diff_eq(DVec3::new(1.0, 2.0, 0.0), EPS));
    }

    #[test]
    fn registration_order_does_not_matter() {
        let run = |first: DVec3, second: DVec3| {
            let mut d = NewtonEuler::new(1);
            d.set_force_sources(vec![Box::new(PushAll(first)), Box::new(PushAll(second))]);
            accel_of(&mut d, 1.0)
        };
        let a = run(DVec3::X, DVec3::Y);
        let b = run(DVec3::Y, DVec3::X);
        assert!(a.abs_diff_eq(b, EPS));
    }

    #[test]
    fn source_applies_to_every_body_with_its_own_mass() {
        // Same force, masses 1 and 4  =>  a = 8 and 2.
        let mut d = NewtonEuler::new(2);
        d.set_force_sources(vec![Box::new(PushAll(DVec3::new(8.0, 0.0, 0.0)))]);
        let mut out = zero_derivs(2);
        d.derivative(&props(&[1.0, 4.0]), &[rest_state(), rest_state()], 0.0, &mut out);
        assert!(out[0].acceleration.abs_diff_eq(DVec3::new(8.0, 0.0, 0.0), EPS));
        assert!(out[1].acceleration.abs_diff_eq(DVec3::new(2.0, 0.0, 0.0), EPS));
    }

    #[test]
    fn constant_gravity_gives_acceleration_g_regardless_of_mass() {
        let g = DVec3::new(0.0, -9.81, 0.0);
        let mut d = NewtonEuler::new(2);
        d.set_force_sources(vec![Box::new(ConstantGravity { field: g })]);
        let mut out = zero_derivs(2);
        d.derivative(&props(&[1.0, 50.0]), &[rest_state(), rest_state()], 0.0, &mut out);
        assert!(out[0].acceleration.abs_diff_eq(g, EPS));
        assert!(out[1].acceleration.abs_diff_eq(g, EPS));
    }

    // ---------- scratch buffer behaviour ----------

    #[test]
    fn each_source_is_evaluated_once_per_derivative_call() {
        let count = Rc::new(Cell::new(0));
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![Box::new(Counter(count.clone()))]);
        let mut out = zero_derivs(1);
        let p = props(&[1.0]);

        d.derivative(&p, &[rest_state()], 0.0, &mut out);
        assert_eq!(count.get(), 1);
        d.derivative(&p, &[rest_state()], 0.0, &mut out);
        assert_eq!(count.get(), 2);
    }

    #[test]
    fn loads_do_not_accumulate_across_calls() {
        // If reset() is skipped, the second call would give double.
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![Box::new(PushAll(DVec3::new(4.0, 0.0, 0.0)))]);
        let mut out = zero_derivs(1);
        let p = props(&[1.0]);

        d.derivative(&p, &[rest_state()], 0.0, &mut out);
        d.derivative(&p, &[rest_state()], 0.0, &mut out);
        assert!(out[0].acceleration.abs_diff_eq(DVec3::new(4.0, 0.0, 0.0), EPS));
    }

    #[test]
    fn same_inputs_give_same_output() {
        let mut d = NewtonEuler::new(1);
        d.set_force_sources(vec![Box::new(PushAll(DVec3::new(1.0, 2.0, 3.0)))]);
        let a = accel_of(&mut d, 2.0);
        let b = accel_of(&mut d, 2.0);
        assert_eq!(a, b);
    }
}
