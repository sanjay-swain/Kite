use glam::DMat3;
use glam::DQuat;
use glam::DVec3;

use crate::system::interactions::Force;
use crate::system::interactions::Torque;

pub struct State {
    pub position: DVec3,
    pub velocity: DVec3,
    pub quaternion: DQuat,
    pub angular_velocity: DVec3,
}

pub struct Body {
    pub id: u32,
    pub mass: f64,
    pub inertia: DMat3,
    pub state: State,
    pub forces: Vec<Force>,
    pub torques: Vec<Torque>,
}
