use glam::DVec3;

use crate::system::constraints::joints::{JacobianRow, Joint};

pub trait BaseConstraint {}

pub struct Constraint {
    pub body_a_index: usize,
    pub body_b_index: usize,

    pub body_a_anchor: DVec3,
    pub body_b_anchor: DVec3,

    pub joint: Box<dyn Joint>,

    pub jacobian: Vec<JacobianRow>,
    pub velocity_bias: Vec<f64>,
}

impl Constraint {
    pub fn new(
        body_a_index: usize,
        body_b_index: usize,
        body_a_anchor: DVec3,
        body_b_anchor: DVec3,
        joint: Box<dyn Joint>,
    ) -> Self {
        let rows = joint.restricted_dof();
        Self {
            body_a_index,
            body_b_index,
            body_a_anchor,
            body_b_anchor,
            joint,
            jacobian: vec![JacobianRow::ZERO; rows],
            velocity_bias: vec![0.0; rows],
        }
    }
}
