use glam::{DQuat, DVec3};

#[derive(Clone, Copy)]
pub struct State {
    pub position: DVec3,
    pub velocity: DVec3,
    pub orientation: DQuat,
    pub angular_velocity: DVec3,
}

impl State {
    pub const ZERO: Self = Self {
        position: DVec3::ZERO,
        velocity: DVec3::ZERO,
        orientation: DQuat::IDENTITY,
        angular_velocity: DVec3::ZERO,
    };

    pub fn is_valid(&self) -> bool {
        self.position.is_finite() && self.velocity.is_finite() && self.orientation.is_finite() && self.angular_velocity.is_finite()
    }
}

use std::fmt;

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "position:         ({:.6}, {:.6}, {:.6})",
            self.position.x, self.position.y, self.position.z
        )?;
        writeln!(
            f,
            "velocity:         ({:.6}, {:.6}, {:.6})",
            self.velocity.x, self.velocity.y, self.velocity.z
        )?;
        writeln!(
            f,
            "orientation:      ({:.6}, {:.6}, {:.6}, {:.6})  [x, y, z, w]",
            self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w
        )?;
        write!(
            f,
            "angular velocity: ({:.6}, {:.6}, {:.6})",
            self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z
        )
    }
}


#[derive(Clone, Copy)]
pub struct StateDerivative {
    pub velocity: DVec3,
    pub acceleration: DVec3,
    pub angular_velocity: DVec3,
    pub angular_acceleration: DVec3,
}

impl StateDerivative {
    pub const ZERO: Self = Self {
        velocity: DVec3::ZERO,
        acceleration: DVec3::ZERO,
        angular_velocity: DVec3::ZERO,
        angular_acceleration: DVec3::ZERO,
    };
}
