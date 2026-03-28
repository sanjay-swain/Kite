use glam::{DMat3, DVec3};

use crate::system::{
    body::Body,
    interactions::{Force, Frame},
    state::State,
};

pub struct World {
    pub bodies: Vec<Body>,
    pub enable_gravity: bool,
    pub gravity: Force,
    next_id: u32,
}

impl World {
    pub fn create_body(&mut self, mass: f64, inertia: DMat3, initial_state: State) {
        self.bodies.push(Body {
            id: self.next_id,
            mass: mass,
            inertia: inertia,
            state: initial_state,
            forces: vec![],
            torques: vec![],
        });
        self.next_id += 1;
    }

    pub fn apply_gravity_force(&mut self) {
        for body in &mut self.bodies {
            body.apply_force(self.gravity);
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self {
            bodies: vec![],
            enable_gravity: true,
            gravity: Force::new(DVec3::new(0.0, 0.0, -9.81), DVec3::ZERO, Frame::Global),
            next_id: 0,
        }
    }
}
