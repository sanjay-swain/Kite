use glam::DVec3;

use crate::system::state::State;

pub struct Jacobian {
    pub v_a: DVec3,
    pub w_a: DVec3,
    pub v_b: DVec3,
    pub w_b: DVec3,
}

pub trait Joint {
    fn calculate_jacobian(state_a: State, state_b: State) -> Jacobian;
}
