use crate::dynamics::Dynamics;
use crate::integrator::euler::ExplicitEuler;
use crate::integrator::euler::SemiImplicitEuler;

use crate::system::properties::Properties;
use crate::system::state::State;

pub mod euler;

pub trait Integrator {
    fn step(&mut self,
            dynamics: &mut Box<dyn Dynamics>,
            properties: &Properties,
            states: &mut [State],
            t: f64,
            dt: f64
    );
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntegratorKind {
    Euler,
    #[default]
    SemiImplicitEuler,
}

impl IntegratorKind {
    pub(crate) fn create(self, n: usize) -> Box<dyn Integrator> {
        match self {
            Self::Euler => Box::new(ExplicitEuler::new(n)),
            Self::SemiImplicitEuler => Box::new(SemiImplicitEuler::new(n)),
        }
    }
}