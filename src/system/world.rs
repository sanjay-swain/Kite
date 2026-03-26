use glam::DMat3;
use glam::DQuat;
use glam::DVec3;

use crate::system::body::Body;
use crate::system::body::State;

pub struct World {
    pub bodies: Vec<Body>,
    pub enable_gravity: bool,
    pub gravity: f64,
    next_id: u32,
}

impl World {
    pub fn create_body(&mut self, mass: f64, inertia: DMat3) {
        self.bodies.push(Body {
            id: self.next_id,
            mass: mass,
            inertia: inertia,
            state: State {
                position: DVec3::ZERO,
                velocity: DVec3::ZERO,
                quaternion: DQuat::IDENTITY,
                angular_velocity: DVec3::ZERO,
            },
        });
        self.next_id += 1;
    }
}
