use glam::DVec3;

use crate::system::{
    body::Body,
    interactions::{Force, Frame},
};

pub trait GravitySolver {
    fn solve(&self, bodies: &mut Vec<Body>);
}

pub struct ConstantGravity {
    pub field: DVec3,
}

impl GravitySolver for ConstantGravity {
    fn solve(&self, bodies: &mut Vec<Body>) {
        for body in bodies {
            body.apply_force(Force {
                force: self.field * body.mass,
                position: DVec3::ZERO,
                frame: Frame::Global,
            });
        }
    }
}

pub struct BasicEarthGravity {}

impl GravitySolver for BasicEarthGravity {
    fn solve(&self, bodies: &mut Vec<Body>) {
        for body in bodies {
            let r = body.state.position.length();
            let constant = -3.986e14 * body.mass / (r * r * r);
            let f = DVec3::new(
                constant * body.state.position.x,
                constant * body.state.position.y,
                constant * body.state.position.z,
            );
            body.apply_force(Force {
                force: f,
                position: DVec3::ZERO,
                frame: Frame::Global,
            });
        }
    }
}
