use glam::DVec3;

use crate::system::constraints::joints::{JacobianRow, Joint};

pub struct ConstraintForce {
    pub f_a: DVec3,
    pub t_a: DVec3,
    pub f_b: DVec3,
    pub t_b: DVec3,
}

impl ConstraintForce {
    pub const ZERO: Self = Self {
        f_a: DVec3::ZERO,
        t_a: DVec3::ZERO,
        f_b: DVec3::ZERO,
        t_b: DVec3::ZERO,
    };
}

pub struct Constraint {
    pub body_a_index: usize,
    pub body_b_index: usize,

    pub body_a_anchor: DVec3,
    pub body_b_anchor: DVec3,

    pub joint: Box<dyn Joint>,

    pub jacobian: [JacobianRow; 6],
    pub velocity_bias: [f64; 6],

    pub constraint_forces: ConstraintForce,

    pub lagrange_multiplier: [f64; 6],
}

impl Constraint {
    pub fn new(
        body_a_index: usize,
        body_b_index: usize,
        body_a_anchor: DVec3,
        body_b_anchor: DVec3,
        joint: Box<dyn Joint>,
    ) -> Self {
        Self {
            body_a_index,
            body_b_index,
            body_a_anchor,
            body_b_anchor,
            joint,
            jacobian: [JacobianRow::ZERO; 6],
            velocity_bias: [0.0; 6],

            constraint_forces: ConstraintForce::ZERO,

            lagrange_multiplier: [0.0; 6],
        }
    }
}
