use glam::{DQuat, DVec3};

/// Specifies the reference frame of a vector
#[derive(Clone, Copy)]
pub enum Frame {
    /// The vector is defined relative to the world's origin
    World,
    /// The vector is defined relative to the body center of mass.
    Body,
}

/// This struct holds information of the force acting on a body.
/// Unit: Newton(N)
#[derive(Clone, Copy)]
pub struct Force {
    pub vector: DVec3,
    /// The point where force is applied.
    /// This is ALWAYS with respect to body frame of reference 
    /// regardless the frame assigned is Body or World
    pub point: DVec3,
    /// This defines whether the force is being applied is oriented according to 
    /// Body frame of reference or World
    pub frame: Frame,
}

impl Force {
    /// Create a new force
    pub fn new(force: DVec3, position: DVec3, frame: Frame) -> Self {
        Self {
            vector: force,
            point: position,
            frame,
        }
    }

    /// Return the force vector in world frame
    /// If the force is already in world frame it will return the same vector
    pub fn to_world(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => self.vector,
            Frame::Body => orientation * self.vector,
        }
    }

    /// Return the force vector in body frame
    /// If the force is already in body frame it will return the same vector
    pub fn to_body(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => orientation.inverse() * self.vector,
            Frame::Body => self.vector,
        }
    }
}

/// The struct holds information of pure torque applied on a body
/// Unit: Newton-meter (Nm)
#[derive(Clone, Copy)]
pub struct Torque {
    /// The torque applied in Nm
    pub torque: DVec3,
    /// Frame of reference about which the torque is being applied
    pub frame: Frame,
}

impl Torque {
    /// Creates a new Torque
    pub fn new(torque: DVec3, frame: Frame) -> Self {
        Self {
            torque,
            frame,
        }
    }

    /// Return the torque vector in world frame of reference
    /// If the torque is already in world frame of reference then return the same.
    pub fn to_world(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => self.torque,
            Frame::Body => orientation * self.torque,
        }
    }

    /// Return the torque vector in body frame of reference
    /// If the torque is already in body frame of reference then return the same
    pub fn to_body(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => orientation.inverse() * self.torque,
            Frame::Body => self.torque,
        }
    }
}

/// Net load on a body
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Load {
    /// Net force, world frame
    force: DVec3,
    /// Net torque about the center of mass, world frame
    torque: DVec3,
}

impl Load {
    pub fn force(&self) -> DVec3 { self.force }
    pub fn torque(&self) -> DVec3 { self.torque }

    /// Adds a force (and the torque it creates about the center of mass).
    pub fn add_force(&mut self, f: Force, orientation: DQuat) {
        self.force += f.to_world(orientation);
        self.torque += (orientation * f.point).cross(f.to_world(orientation));
    }

    /// Adds a pure torque.
    pub fn add_torque(&mut self, t: Torque, orientation: DQuat) {
        self.torque += t.to_world(orientation);
    }

    pub fn reset(&mut self) {
        self.force = DVec3::ZERO;
        self.torque = DVec3::ZERO;
    }
}

#[cfg(test)]
mod tests {
    use crate::system::loads::{Force, Frame, Torque};
    use glam::{DQuat, DVec3};
    use std::f64::consts::PI;

    #[test]
    fn identity() {
        let f = Force::new(DVec3::new(10.0, 20.0, 30.0), DVec3::ZERO, Frame::Body);
        let tau = Torque::new(DVec3::new(10.0, 20.0, 30.0), Frame::Body);

        let orientation = DQuat::IDENTITY;
        assert!((f.to_world(orientation).x - f.vector.x).abs() < 1e-6);
        assert!((f.to_world(orientation).y - f.vector.y).abs() < 1e-6);
        assert!((f.to_world(orientation).z - f.vector.z).abs() < 1e-6);

        assert!((tau.to_world(orientation).x - tau.torque.x).abs() < 1e-6);
        assert!((tau.to_world(orientation).y - tau.torque.y).abs() < 1e-6);
        assert!((tau.to_world(orientation).z - tau.torque.z).abs() < 1e-6);
    }

    #[test]
    fn body_to_world() {
        let f = Force::new(DVec3::new(10.0, 20.0, 30.0), DVec3::ZERO, Frame::Body);
        let tau = Torque::new(DVec3::new(10.0, 20.0, 30.0), Frame::Body);

        let orientation = DQuat::from_axis_angle(DVec3::X, PI / 2.0);

        assert!((f.to_world(orientation).x - 10.0).abs() < 1e-6);
        assert!((f.to_world(orientation).y + 30.0).abs() < 1e-6);
        assert!((f.to_world(orientation).z - 20.0).abs() < 1e-6);

        assert!((tau.to_world(orientation).x - 10.0).abs() < 1e-6);
        assert!((tau.to_world(orientation).y + 30.0).abs() < 1e-6);
        assert!((tau.to_world(orientation).z - 20.0).abs() < 1e-6);
    }

    #[test]
    fn world_to_body() {
        let f = Force::new(DVec3::new(10.0, -30.0, 20.0), DVec3::ZERO, Frame::World);
        let tau = Torque::new(DVec3::new(10.0, -30.0, 20.0), Frame::World);

        let orientation = DQuat::from_axis_angle(DVec3::X, PI / 2.0);

        assert!((f.to_body(orientation).x - 10.0).abs() < 1e-6);
        assert!((f.to_body(orientation).y - 20.0).abs() < 1e-6);
        assert!((f.to_body(orientation).z - 30.0).abs() < 1e-6);

        assert!((tau.to_body(orientation).x - 10.0).abs() < 1e-6);
        assert!((tau.to_body(orientation).y - 20.0).abs() < 1e-6);
        assert!((tau.to_body(orientation).z - 30.0).abs() < 1e-6);
    }
}