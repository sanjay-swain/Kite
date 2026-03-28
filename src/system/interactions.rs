use glam::{DQuat, DVec3};

/// Specifies the reference frame of a vector
#[derive(Clone, Copy)]
pub enum Frame {
    /// The vector is defined relative to the world's origin
    Global,
    /// The vector is defined relative to the body center of mass.
    Local,
}

#[derive(Clone, Copy)]
pub struct Force {
    pub force: DVec3,
    /// The point where force is applied.
    /// This is ALWAYS with respect to local frame of reference regardless the frame assigned is Local or Global
    pub position: DVec3,
    /// This defines whether the force is being applied is oriented according to local frame of reference or global
    pub frame: Frame,
}

impl Force {
    pub const ZERO_LOCAL: Self = Self {
        force: DVec3::ZERO,
        position: DVec3::ZERO,
        frame: Frame::Local,
    };

    pub const ZERO_GLOBAL: Self = Self {
        force: DVec3::ZERO,
        position: DVec3::ZERO,
        frame: Frame::Global,
    };

    pub fn new(force: DVec3, position: DVec3, frame: Frame) -> Self {
        Self {
            force: force,
            position: position,
            frame: frame,
        }
    }

    pub fn to_global(&self, orientation: DQuat) -> DVec3 {
        return match self.frame {
            Frame::Global => self.force,
            Frame::Local => orientation * self.force,
        };
    }

    pub fn to_local(&self, orientation: DQuat) -> DVec3 {
        return match self.frame {
            Frame::Global => orientation.inverse() * self.force,
            Frame::Local => self.force,
        };
    }
}

#[derive(Clone, Copy)]
pub struct Torque {
    pub torque: DVec3,
    pub frame: Frame,
}

impl Torque {
    pub const ZERO_LOCAL: Self = Self {
        torque: DVec3::ZERO,
        frame: Frame::Local,
    };

    pub const ZERO_GLOBAL: Self = Self {
        torque: DVec3::ZERO,
        frame: Frame::Global,
    };

    pub fn new(torque: DVec3, frame: Frame) -> Self {
        Self {
            torque: torque,
            frame: frame,
        }
    }

    pub fn to_global(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::Global => self.torque,
            Frame::Local => orientation * self.torque,
        }
    }

    pub fn to_local(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::Global => orientation.inverse() * self.torque,
            Frame::Local => self.torque,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn identity() {
        let f = Force::new(DVec3::new(10.0, 20.0, 30.0), DVec3::ZERO, Frame::Local);
        let tau = Torque::new(DVec3::new(10.0, 20.0, 30.0), Frame::Local);

        let orientation = DQuat::IDENTITY;
        assert!((f.to_global(orientation).x - f.force.x) < 1e-6);
        assert!((f.to_global(orientation).y - f.force.y) < 1e-6);
        assert!((f.to_global(orientation).z - f.force.z) < 1e-6);

        assert!((tau.to_global(orientation).x - tau.torque.x).abs() < 1e-6);
        assert!((tau.to_global(orientation).y - tau.torque.y).abs() < 1e-6);
        assert!((tau.to_global(orientation).z - tau.torque.z).abs() < 1e-6);
    }

    #[test]
    fn local_to_global() {
        let f = Force::new(DVec3::new(10.0, 20.0, 30.0), DVec3::ZERO, Frame::Local);
        let tau = Torque::new(DVec3::new(10.0, 20.0, 30.0), Frame::Local);

        let orientation = DQuat::from_axis_angle(DVec3::X, PI / 2.0);

        assert!((f.to_global(orientation).x - 10.0).abs() < 1e-6);
        assert!((f.to_global(orientation).y + 30.0).abs() < 1e-6);
        assert!((f.to_global(orientation).z - 20.0).abs() < 1e-6);

        assert!((tau.to_global(orientation).x - 10.0).abs() < 1e-6);
        assert!((tau.to_global(orientation).y + 30.0).abs() < 1e-6);
        assert!((tau.to_global(orientation).z - 20.0).abs() < 1e-6);
    }

    #[test]
    fn global_to_local() {
        let f = Force::new(DVec3::new(10.0, -30.0, 20.0), DVec3::ZERO, Frame::Global);
        let tau = Torque::new(DVec3::new(10.0, -30.0, 20.0), Frame::Global);

        let orientation = DQuat::from_axis_angle(DVec3::X, PI / 2.0);

        assert!((f.to_local(orientation).x - 10.0).abs() < 1e-6);
        assert!((f.to_local(orientation).y - 20.0).abs() < 1e-6);
        assert!((f.to_local(orientation).z - 30.0).abs() < 1e-6);

        assert!((tau.to_local(orientation).x - 10.0).abs() < 1e-6);
        assert!((tau.to_local(orientation).y - 20.0).abs() < 1e-6);
        assert!((tau.to_local(orientation).z - 30.0).abs() < 1e-6);
    }
}
