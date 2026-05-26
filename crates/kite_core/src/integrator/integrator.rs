use crate::system::body::Body;

pub trait Integrator {
    fn step(&self, bodies: &mut Vec<Body>, step_size: f64);
}
