use glam::DQuat;
use crate::dynamics::Dynamics;
use crate::integrator::Integrator;
use crate::system::properties::Properties;
use crate::system::state::{State, StateDerivative};

pub struct ExplicitEuler {
    state_derivatives: Vec<StateDerivative>
}

impl ExplicitEuler {
    pub fn new(n: usize) -> Self {
        Self {
            state_derivatives: vec![StateDerivative::ZERO; n],
        }
    }
}

impl Integrator for ExplicitEuler {
    fn step(&mut self,
            dynamics: &mut Box<dyn Dynamics>,
            properties: &Properties,
            states: &mut [State],
            t: f64,
            dt: f64
    ) {
        // Update the state_derivatives
        dynamics.derivative(properties, &states, t, &mut self.state_derivatives);

        for i in 0..states.len() {
            states[i].position += states[i].velocity * dt;
            states[i].velocity += self.state_derivatives[i].acceleration * dt;

            let q = states[i].orientation;
            let w = states[i].angular_velocity;
            let dq_dt = DQuat::from_xyzw(
                0.5 * (w.x * q.x + w.y * q.z - w.z * q.y),
                0.5 * (-w.x * q.z + w.y * q.w + w.z * q.x),
                0.5 * (w.x * q.y - w.y * q.x + w.z * q.w),
                0.5 * (-w.x * q.x - w.y * q.y - w.z * q.z),
            );

            states[i].orientation = (q + dq_dt * dt).normalize();
            states[i].angular_velocity +=
                self.state_derivatives[i].angular_acceleration * dt;
        }
    }
}

pub struct SemiImplicitEuler {
    state_derivatives: Vec<StateDerivative>
}

impl SemiImplicitEuler {
    pub fn new(n: usize) -> Self {
        Self {
            state_derivatives: vec![StateDerivative::ZERO; n],
        }
    }
}

impl Integrator for SemiImplicitEuler {
    fn step(&mut self,
            dynamics: &mut Box<dyn Dynamics>,
            properties: &Properties,
            states: &mut [State],
            t: f64,
            dt: f64
    ) {
        // Update the state_derivatives
        dynamics.derivative(properties, &states, t, &mut self.state_derivatives);

        
        for i in 0..states.len() {
            states[i].velocity += self.state_derivatives[i].acceleration * dt;
            states[i].position += states[i].velocity * dt;

            states[i].angular_velocity += self.state_derivatives[i].angular_acceleration * dt;

            let angle = states[i].angular_velocity.length() * dt;
            if angle > 1e-6 {
                let axis = states[i].angular_velocity.normalize();
                let dq = DQuat::from_axis_angle(axis, angle);

                states[i].orientation = (states[i].orientation * dq).normalize();
            }
        }
    }
}
