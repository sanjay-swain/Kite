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

// TODO: Write Tests Again
// #[cfg(test)]
// mod tests {
//     use crate::dynamics::forces::types::{Force, Frame, Torque};
//     use glam::{DMat3, DVec3};
//
//     use super::*;
//
//     #[test]
//     fn translation() {
//         let mut bodies: Vec<Body> = Vec::new();
//         bodies.push(Body {
//             id: 0,
//             mass: 5.0,
//             mass_inv: 1.0 / 5.0,
//             ..Default::default()
//         });
//
//         bodies[0].apply_force(Force::new(
//             DVec3::new(50.0, 0.0, 0.0),
//             DVec3::ZERO,
//             Frame::Body,
//         ));
//
//         let solver = NewtonEuler {};
//         solver.solve(&mut bodies);
//
//         assert!((bodies[0].state_derivative.acceleration.x - 10.0).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.acceleration.y).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.acceleration.z).abs() < 1e-6);
//
//         assert!((bodies[0].state_derivative.angular_acceleration.x).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.angular_acceleration.y).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.angular_acceleration.z).abs() < 1e-6);
//     }
//
//     #[test]
//     fn rotation() {
//         let inertia = DMat3::from_diagonal(DVec3 {
//             x: 10.0,
//             y: 20.0,
//             z: 30.0,
//         });
//         let mut bodies: Vec<Body> = Vec::new();
//         bodies.push(Body {
//             id: 0,
//             mass: 1.0,
//             inertia: inertia,
//             inertia_inv: inertia.inverse(),
//             ..Default::default()
//         });
//
//         bodies[0].apply_torque(Torque::new(DVec3::new(0.0, 20.0, 0.0), Frame::Body));
//
//         let solver = NewtonEuler {};
//         solver.solve(&mut bodies);
//
//         assert!((bodies[0].state_derivative.acceleration.x).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.acceleration.y).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.acceleration.z).abs() < 1e-6);
//
//         assert!((bodies[0].state_derivative.angular_acceleration.x).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.angular_acceleration.y - 1.0).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.angular_acceleration.z).abs() < 1e-6);
//     }
//
//     #[test]
//     fn gyroscope() {
//         let inertia = DMat3::from_diagonal(DVec3 {
//             x: 10.0,
//             y: 20.0,
//             z: 30.0,
//         });
//         let mut bodies: Vec<Body> = vec![Body {
//             id: 0,
//             mass: 1.0,
//             inertia: inertia,
//             inertia_inv: inertia.inverse(),
//             ..Default::default()
//         }];
//
//         bodies[0].state.angular_velocity = DVec3 {
//             x: 1.0,
//             y: 0.0,
//             z: 1.0,
//         };
//
//         let solver = NewtonEuler {};
//         solver.solve(&mut bodies);
//
//         assert!((bodies[0].state_derivative.angular_acceleration.x).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.angular_acceleration.y - 1.0).abs() < 1e-6);
//         assert!((bodies[0].state_derivative.angular_acceleration.z).abs() < 1e-6);
//     }
// }
