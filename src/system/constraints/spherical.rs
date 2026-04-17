use glam::DVec3;

use crate::system::{
    constraints::joints::{JacobianRow, Joint},
    state::State,
};

pub struct SphericalJoint {}

impl Joint for SphericalJoint {
    fn restricted_dof(&self) -> usize {
        return 3;
    }

    fn calculate_jacobian(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        jacobian: &mut [JacobianRow; 6],
    ) {
        let r_a = state_a.orientation.mul_vec3(anchor_a);
        let r_b = state_b.orientation.mul_vec3(anchor_b);

        jacobian[0] = JacobianRow {
            v_a: DVec3::new(-1.0, 0.0, 0.0),
            w_a: DVec3::new(0.0, -r_a.z, r_a.y),
            v_b: DVec3::new(1.0, 0.0, 0.0),
            w_b: DVec3::new(0.0, r_b.z, -r_b.y),
        };

        jacobian[1] = JacobianRow {
            v_a: DVec3::new(0.0, -1.0, 0.0),
            w_a: DVec3::new(r_a.z, 0.0, -r_a.x),
            v_b: DVec3::new(0.0, 1.0, 0.0),
            w_b: DVec3::new(-r_b.z, 0.0, r_b.x),
        };

        jacobian[2] = JacobianRow {
            v_a: DVec3::new(0.0, 0.0, -1.0),
            w_a: DVec3::new(-r_a.y, r_a.x, 0.0),
            v_b: DVec3::new(0.0, 0.0, 1.0),
            w_b: DVec3::new(r_b.y, -r_b.x, 0.0),
        };
    }

    fn calculate_velocity_bias(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        velocity_bias: &mut [f64; 6],
    ) {
        let r_a = state_a.orientation.mul_vec3(anchor_a);
        let r_b = state_b.orientation.mul_vec3(anchor_b);
        let w_a = state_a.angular_velocity;
        let w_b = state_b.angular_velocity;
        velocity_bias[0] = (-w_a.x * (w_a.z * r_a.z + w_a.y * r_a.y)
            + w_a.y * w_a.y * r_a.x
            + w_a.z * w_a.z * r_a.x)
            - (-w_b.x * (w_b.z * r_b.z + w_b.y * r_b.y)
                + w_b.y * w_b.y * r_b.x
                + w_b.z * w_b.z * r_b.x);

        velocity_bias[1] = (w_a.x * w_a.x * r_a.y - w_a.y * (w_a.z * r_a.z + w_a.x * r_a.x)
            + w_a.z * w_a.z * r_a.x)
            - (w_b.x * w_b.x * r_b.y - w_b.y * (w_b.z * r_b.z + w_b.x * r_b.x)
                + w_b.z * w_b.z * r_b.x);

        velocity_bias[2] = (w_a.x * w_a.x * r_a.z + w_a.y * w_a.y * r_a.z
            - w_a.z * (w_a.y * r_a.y + w_a.x * r_a.x))
            - (w_b.x * w_b.x * r_b.z + w_b.y * w_b.y * r_b.z
                - w_b.z * (w_b.y * r_b.y + w_b.x * r_b.x));
    }
}
