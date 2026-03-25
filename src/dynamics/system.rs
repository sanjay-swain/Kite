use nalgebra as na;

pub struct State {
    position: na::Vector3<f64>,
    velocity: na::Vector3<f64>,
    quaternion: na::UnitQuaternion<f64>,
    angular_velocity: na::Vector3<f64>,
}

pub struct Body {
    mass: f64,
    inertia: na::Matrix3<f64>,
    state: State,
}

pub struct World {
    bodies: Vec<Body>,
    enable_gravity: bool,
    gravity: f64,
}

impl World {
    fn create_body(mass: f64, inertia: na::Matrix3<f64>) -> Body {
        // TODO: Needs to be updated to automatically append created bodies to world instance
        Body {
            mass: mass,
            inertia: inertia,
            state: State {
                position: na::Vector3::<f64>::zeros(),
                velocity: na::Vector3::<f64>::zeros(),
                quaternion: na::UnitQuaternion::identity(),
                angular_velocity: na::Vector3::<f64>::zeros(),
            },
        }
    }
}
