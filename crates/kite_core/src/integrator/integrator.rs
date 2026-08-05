use crate::system::body::Body;

pub trait Integrator: Sized {
    fn new(step_size: f64) -> Self;
    fn step(&self, bodies: &mut Vec<Body>);
}
