use crate::{error::PhysicsError, system::body::Body};

pub trait Integrator: Sized {
    fn new(step_size: f64) -> Result<Self, PhysicsError>;
    fn step(&self, bodies: &mut Vec<Body>);
}
