use serde::Serialize;

use crate::system::{body::Body, constraints::constraint::Constraint};

#[derive(Serialize)]
pub struct PhysicsLog {
    pub time: f64,

    pub pos_x: f64,
    pub pos_y: f64,
    pub pos_z: f64,

    pub vel_x: f64,
    pub vel_y: f64,
    pub vel_z: f64,

    pub ori_x: f64,
    pub ori_y: f64,
    pub ori_z: f64,

    pub constraint_error: f64, // Important for checking solver health

    pub force_a_x: f64,
    pub force_a_y: f64,
    pub force_a_z: f64,

    pub force_b_x: f64,
    pub force_b_y: f64,
    pub force_b_z: f64,

    pub torque_a_x: f64,
    pub torque_a_y: f64,
    pub torque_a_z: f64,

    pub torque_b_x: f64,
    pub torque_b_y: f64,
    pub torque_b_z: f64,

    pub energy: f64,
}

impl PhysicsLog {
    pub const ZERO: Self = Self {
        time: 0.0,

        pos_x: 0.0,
        pos_y: 0.0,
        pos_z: 0.0,

        vel_x: 0.0,
        vel_y: 0.0,
        vel_z: 0.0,

        ori_x: 0.0,
        ori_y: 0.0,
        ori_z: 0.0,

        constraint_error: 0.0, // Important for checking solver health

        force_a_x: 0.0,
        force_a_y: 0.0,
        force_a_z: 0.0,

        force_b_x: 0.0,
        force_b_y: 0.0,
        force_b_z: 0.0,

        torque_a_x: 0.0,
        torque_a_y: 0.0,
        torque_a_z: 0.0,

        torque_b_x: 0.0,
        torque_b_y: 0.0,
        torque_b_z: 0.0,

        energy: 0.0,
    };

    pub fn update(&mut self, body: &Body, constraint: &Constraint, time: f64) {
        self.time = time;

        (self.pos_x, self.pos_y, self.pos_z) = (
            body.state.position.x,
            body.state.position.y,
            body.state.position.z,
        );

        (self.vel_x, self.vel_y, self.vel_z) = (
            body.state.velocity.x,
            body.state.velocity.y,
            body.state.velocity.z,
        );

        (self.ori_x, self.ori_y, self.ori_z) = body.state.orientation.to_euler(glam::EulerRot::XYZ);

        (self.force_a_x, self.force_a_y, self.force_a_z) = (
            constraint.constraint_forces[0],
            constraint.constraint_forces[1],
            constraint.constraint_forces[2],
        );

        (self.torque_a_x, self.torque_a_y, self.torque_a_z) = (
            constraint.constraint_forces[3],
            constraint.constraint_forces[4],
            constraint.constraint_forces[5],
        );

        (self.force_b_x, self.force_b_y, self.force_b_z) = (
            constraint.constraint_forces[6],
            constraint.constraint_forces[7],
            constraint.constraint_forces[8],
        );

        (self.torque_b_x, self.torque_b_y, self.torque_b_z) = (
            constraint.constraint_forces[9],
            constraint.constraint_forces[10],
            constraint.constraint_forces[11],
        );
    }
}
