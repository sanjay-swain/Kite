use crate::dynamics::newton_euler::NewtonEuler;
use crate::dynamics::sources::ForceSource;
use crate::system::properties::Properties;
use crate::system::state::{State, StateDerivative};


pub mod gauss_seidel;
pub mod newton_euler;
pub mod sources;

pub trait Dynamics {
    fn set_force_sources(&mut self, sources: Vec<Box<dyn ForceSource>>);

    fn derivative(&mut self,
                  properties: &Properties,
                  states: &[State],
                  t: f64,
                  state_derivatives: &mut [StateDerivative]
    );
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DynamicsKind {
    #[default]
    NewtonEuler,
}

impl DynamicsKind {
    pub(crate) fn create(self, n: usize) -> Box<dyn Dynamics> {
        match self { DynamicsKind::NewtonEuler => Box::new(NewtonEuler::new(n)) }
    }
}