use glam::DVec3;

use crate::system::state::State;

#[derive(Clone, Copy)]
pub struct JacobianRow {
    pub v_a: DVec3,
    pub w_a: DVec3,
    pub v_b: DVec3,
    pub w_b: DVec3,
}

impl JacobianRow {
    pub const ZERO: Self = Self {
        v_a: DVec3::ZERO,
        w_a: DVec3::ZERO,
        v_b: DVec3::ZERO,
        w_b: DVec3::ZERO,
    };
}

pub trait Joint {
    fn restricted_dof(&self) -> usize;

    fn calculate_jacobian(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        jacobian: &mut Vec<JacobianRow>,
    );

    fn calculate_velocity_bias(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        velocity_bias: &mut Vec<f64>,
    );
}
