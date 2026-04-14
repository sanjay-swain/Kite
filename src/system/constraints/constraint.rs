use glam::DVec3;

use crate::system::constraints::joints::{JacobianRow, Joint};

pub struct Constraint<JointType: Joint> {
    pub body_a_index: usize,
    pub body_a_id: usize,
    pub body_b_index: usize,
    pub body_b_id: usize,

    pub body_a_anchor: DVec3,
    pub body_b_anchor: DVec3,

    pub joint: JointType,

    pub jacobian: Vec<JacobianRow>,
    pub velocity_bias: Vec<f64>,
}

impl<JointType: Joint> Constraint<JointType> {
    pub fn new(
        body_a_index: usize,
        body_a_id: usize,
        body_b_index: usize,
        body_b_id: usize,
        body_a_anchor: DVec3,
        body_b_anchor: DVec3,
        joint: JointType,
    ) -> Self {
        let rows = joint.restricted_dof();
        Self {
            body_a_index,
            body_a_id,
            body_b_index,
            body_b_id,
            body_a_anchor,
            body_b_anchor,
            joint,
            jacobian: vec![JacobianRow::ZERO; rows],
            velocity_bias: vec![0.0; rows],
        }
    }
}
