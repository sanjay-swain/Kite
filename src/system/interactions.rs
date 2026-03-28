use glam::DVec3;

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
    /// The point where force is applied is ALWAYS with respect to local frame of reference
    pub position: DVec3,
    /// This defines whether the force is being applied is oriented according to local frame of reference or global
    pub frame: Frame,
}

impl Force {
    pub fn new(force: DVec3, position: DVec3, frame: Frame) -> Self {
        Self {
            force: force,
            position: position,
            frame: frame,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Torque {
    pub torque: DVec3,
    pub frame: Frame,
}
