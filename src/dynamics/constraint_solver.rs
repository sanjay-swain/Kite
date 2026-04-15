use crate::system::{
    constraints::constraint::Constraint,
    interactions::{Force, Torque},
};

pub struct ConstraintForce {
    f_a: Force,
    t_a: Torque,
    f_b: Force,
    t_b: Torque,
}

pub trait ConstraintSolver {
    fn solve(&self, constraints: &mut Vec<Constraint>) -> ConstraintForce;
}

pub struct HardConstraint {}

impl ConstraintSolver for HardConstraint {
    fn solve(&self, constraint: &mut Vec<Constraint>) -> ConstraintForce {
        let constraint_force: ConstraintForce = ConstraintForce {
            f_a: Force::ZERO_GLOBAL,
            t_a: Torque::ZERO_GLOBAL,
            f_b: Force::ZERO_GLOBAL,
            t_b: Torque::ZERO_GLOBAL,
        };

        return constraint_force;
    }
}
