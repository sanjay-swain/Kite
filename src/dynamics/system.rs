use nalgebra as na;

pub struct State {
    position: na::Vector3<f64>,
    velocity: na::Vector3<f64>,
    quaternion: na::UnitQuaternion<f64>,
    angular_velocity: na::Vector3<f64>,
}

pub struct Body {
    id: u32,
    mass: f64,
    inertia: na::Matrix3<f64>,
    state: State,
}

pub struct World {
    bodies: Vec<Body>,
    enable_gravity: bool,
    gravity: f64,
    next_id: u32,
}

impl World {
    pub fn create_body(&mut self, mass: f64, inertia: na::Matrix3<f64>) {
        self.bodies.push(Body {
            id: self.next_id,
            mass: mass,
            inertia: inertia,
            state: State {
                position: na::Vector3::<f64>::zeros(),
                velocity: na::Vector3::<f64>::zeros(),
                quaternion: na::UnitQuaternion::identity(),
                angular_velocity: na::Vector3::<f64>::zeros(),
            },
        });
        self.next_id += 1;
    }
}
