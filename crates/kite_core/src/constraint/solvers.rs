pub mod acceleration_solver;
pub mod soft_constraint;

use crate::constraint::constraint::Constraint;
use crate::constraint::solvers::acceleration_solver::AccelerationConstraint;

pub trait ConstraintSolver {
    fn apply_constraint_force(&self, constraint: &mut Constraint);

    //TODO: Implement these
    fn correct(&self);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConstraintSolverKind{
    AccelerationConstraint,
    SoftConstraint,
    #[default]
    NoSolver
}

impl ConstraintSolverKind {
    pub(crate) fn create(self, n: usize) -> Box<dyn ConstraintSolver> {
        match self {
            Self::AccelerationConstraint => Box::new(AccelerationConstraint::new()),
            Self::SoftConstraint => Box::new(AccelerationConstraint::new()),
            Self::NoSolver => Box::new(AccelerationConstraint::new()),
        }
    }
}