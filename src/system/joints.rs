use glam::DVec3;

pub struct Jacobian {
    pub v_a: DVec3,
    pub w_a: DVec3,
    pub v_b: DVec3,
    pub w_b: DVec3,
}

pub trait Joint {
    fn calculate_jacobian() -> Jacobian;
}
