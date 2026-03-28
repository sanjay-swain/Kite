use glam::DVec3;

use crate::system::{
    body::Body,
    interactions::{Force, Frame, Torque},
};

pub fn compute_forces(body: &Body) -> Force {
    let mut resultant_force: Force = Force::new(DVec3::ZERO, DVec3::ZERO, Frame::Global);

    for force in &body.forces {
        // If local convert them to global and then add the force
        match force.frame {
            Frame::Global => resultant_force.force += force.force,
            Frame::Local => resultant_force.force += body.state.orientation * force.force,
        };
    }

    return resultant_force;
}

pub fn compute_torqes(body: &Body) -> Torque {
    let mut resultant_torque: Torque = Torque::new(DVec3::ZERO, Frame::Local);

    for torque in &body.torques {
        // Sum torques in local frame of refenrece
        match torque.frame {
            Frame::Global => {
                resultant_torque.torque += body.state.orientation.inverse() * torque.torque
            }
            Frame::Local => resultant_torque.torque += torque.torque,
        }
    }

    for force in &body.forces {
        // If Global we must convert the force into local
        match force.frame {
            Frame::Global => {
                resultant_torque.torque += force
                    .position
                    .cross(body.state.orientation.inverse() * force.force)
            }
            Frame::Local => resultant_torque.torque += force.position.cross(force.force),
        }
    }

    return resultant_torque;
}
