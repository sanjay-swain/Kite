use glam::DMat3;
use std::fmt::{self};

#[derive(Debug)]
pub enum PhysicsError {
    InvalidMass(f64),
    InvalidInertia(DMat3),
    InvalidStepSize(f64),
}

impl fmt::Display for PhysicsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PhysicsError::InvalidInertia(_) => {
                write!(f, "The inertia matrix is invalid")
            }

            PhysicsError::InvalidMass(_) => {
                write!(f, "The mass can't be negative or zero")
            }

            PhysicsError::InvalidStepSize(_) => {
                write!(f, "The integrator step size must be positive")
            }
        }
    }
}

impl std::error::Error for PhysicsError {}
