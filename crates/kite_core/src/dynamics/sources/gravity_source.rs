use crate::{
    dynamics::sources::ForceSource,
    system::{
        loads::{Force, Frame, Load},
        properties::{
            BodyId,
            Properties
        },
        state::State
    }
};
use glam::{DQuat, DVec3};

pub struct ConstantGravity {
    pub field: DVec3,
}

impl ConstantGravity {
    pub fn new(field: DVec3) -> Self {
        ConstantGravity { field }
    }
}

impl ForceSource for ConstantGravity {
    fn compute_loads(&self, properties: &Properties, states: &[State], _t: f64, loads: &mut [Load]) {
        for i in 0..states.len() {
            loads[i].add_force(Force {
                vector: self.field * properties.mass(BodyId::new(i)),
                point: DVec3::ZERO,
                frame: Frame::World,
            },
            DQuat::IDENTITY)
        }
    }
}

pub struct BasicEarthGravity {}

impl BasicEarthGravity {
    pub fn new() -> Self {
        BasicEarthGravity {}
    }
}

impl ForceSource for BasicEarthGravity {
    fn compute_loads(&self, properties: &Properties, states: &[State], _t: f64, loads: &mut [Load]) {
        for i in 0..states.len() {
            let r = states[i].position.length();
            let constant = -3.986e14 * properties.mass(BodyId::new(i)) / (r * r * r);
            let f = DVec3::new(
                constant * states[i].position.x,
                constant * states[i].position.y,
                constant * states[i].position.z,
            );
            loads[i].add_force(Force {
                vector: f,
                point: DVec3::ZERO,
                frame: Frame::World,
            }, DQuat::IDENTITY)
        }
    }
}
