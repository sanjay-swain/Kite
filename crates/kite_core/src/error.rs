use glam::DMat3;
use std::fmt::{self};

#[derive(Debug)]
pub enum PhysicsError {
    InvalidMass(f64),
    InvalidInertia(DMat3),
    InvalidFrequency(u64),
    InsufficientData,
    InvalidState,
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

            PhysicsError::InvalidFrequency(_) => {
                write!(f, "The integrator step frequency is too high or 0")
            }
            
            PhysicsError::InsufficientData => {
                write!(f, "Not enough data")
            }

            PhysicsError::InvalidState => {
                write!(f, "The state of a body is not finite")
            }
        }
    }
}

impl std::error::Error for PhysicsError {}
