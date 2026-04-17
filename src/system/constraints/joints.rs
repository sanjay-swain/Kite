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

    pub fn dot(&self, jacobian_row: &JacobianRow) -> f64 {
        return self.v_a.dot(jacobian_row.v_a)
            + self.w_a.dot(jacobian_row.w_a)
            + self.w_b.dot(jacobian_row.w_b)
            + self.v_b.dot(jacobian_row.v_b);
    }
}

pub trait Joint {
    fn restricted_dof(&self) -> usize;

    fn calculate_jacobian(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        jacobian: &mut [JacobianRow; 6],
    );

    fn calculate_velocity_bias(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        velocity_bias: &mut [f64; 6],
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_product_tests() {
        let zero_jacobian = JacobianRow::ZERO;
        let ones_jacobian = JacobianRow {
            v_a: DVec3::ONE,
            w_a: DVec3::ONE,
            v_b: DVec3::ONE,
            w_b: DVec3::ONE,
        };
        let complex_jacobian_1 = JacobianRow {
            v_a: DVec3::new(1.0, -2.0, 3.0),
            w_a: DVec3::new(0.0, 5.0, -1.0),
            v_b: DVec3::new(0.5, 2.0, -3.0),
            w_b: DVec3::new(1.0, 0.0, -4.0),
        };

        let complex_jacobian_2 = JacobianRow {
            v_a: DVec3::new(2.0, 3.0, -1.0),
            w_a: DVec3::new(5.0, 0.0, 4.0),
            v_b: DVec3::new(2.0, -1.0, 0.0),
            w_b: DVec3::new(3.0, 6.0, 1.0),
        };

        assert!((zero_jacobian.dot(&zero_jacobian)).abs() < 1e-6);
        assert!((ones_jacobian.dot(&zero_jacobian)).abs() < 1e-6);
        assert!((ones_jacobian.dot(&ones_jacobian) - 12.0).abs() < 1e-6);
        assert!((complex_jacobian_1.dot(&complex_jacobian_2) - (-13.0)).abs() < 1e-6);
    }
}
