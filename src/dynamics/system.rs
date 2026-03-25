use nalgebra as nl;

pub struct State {
    position: nl::Vector3<f64>,
    velocity: nl::Vector3<f64>,
    quaternion: nl::UnitQuaternion<f64>,
    angular_velocity: nl::Vector3<f64>,
}

pub struct Body {
    mass: f64,
    inertia: nl::Matrix3<f64>,
    state: State,
}

pub struct World {
    bodies: Vec<Body>,
    enable_gravity: bool,
}
