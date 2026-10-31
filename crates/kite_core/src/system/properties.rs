use crate::error::PhysicsError;
use glam::DMat3;

/// Hold all the Properties of all bodies.
pub struct Properties {
    masses: Vec<f64>,
    masses_inverse: Vec<f64>,
    inertias: Vec<DMat3>,               // body frame
    inertias_inverse: Vec<DMat3>,        // body frame
}

impl Properties {
    pub fn new(masses: Vec<f64>, inertias: Vec<DMat3>) -> Result<Self, PhysicsError> {
        let mut masses_inverse: Vec<f64> = Vec::new();
        let mut inertias_inverse: Vec<DMat3> = Vec::new();
        // Check if the length match
        if masses.len() != inertias.len() {
            return Err(PhysicsError::InsufficientData)
        }

        // Validate mass > 0
        for mass in &masses {
            if !(*mass > 0.0) {
                return Err(PhysicsError::InvalidMass(*mass))
            } else {
                masses_inverse.push(1.0/mass);
            }
        }

        // Validate the inertia matrix
        for inertia in &inertias {
            // 1. All diagonal elements must be greater than zero
            let diagonal = inertia.diagonal();
            if (diagonal.x <= 0.0) || (diagonal.y <= 0.0) || (diagonal.z <= 0.0) {
                return Err(PhysicsError::InvalidInertia(*inertia));
            }

            // 2. The product of inertia of any two product of inertia must be
            // greater than the third product of inertia
            if inertia.x_axis.y + inertia.x_axis.z < inertia.y_axis.z
                || (inertia.x_axis.y + inertia.y_axis.z < inertia.x_axis.z)
                || (inertia.y_axis.z + inertia.x_axis.z < inertia.x_axis.y)
            {
                return Err(PhysicsError::InvalidInertia(*inertia));
            }

            // 3. Inertia matrix must be symmetric
            if !((inertia.x_axis.y - inertia.y_axis.x).abs() < 1e-6
                && (inertia.x_axis.z - inertia.z_axis.x).abs() < 1e-6
                && (inertia.z_axis.y - inertia.y_axis.z).abs() < 1e-6)
            {
                return Err(PhysicsError::InvalidInertia(*inertia));
            }

            inertias_inverse.push(inertia.inverse());
        }

        Ok(Self {
            masses,
            masses_inverse,
            inertias,
            inertias_inverse
        })
    }

    /// Get the mass of a specified body
    pub fn mass(&self, body_id: BodyId) -> f64 { self.masses[body_id.index()] }
    /// Get the mass_inverse of a specified body
    pub fn mass_inverse(&self, body_id: BodyId) -> f64 { self.masses_inverse[body_id.index()] }
    /// Get the inertia matrix of a specified body
    pub fn inertia(&self, body_id: BodyId) -> DMat3 { self.inertias[body_id.index()] }
    /// Get the inertia_inverse matrix of a specified body
    pub fn inertia_inverse(&self, body_id: BodyId) -> DMat3 { self.inertias_inverse[body_id.index()] }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyId(usize);

impl BodyId {
    pub(crate) fn new(id: usize) -> Self { BodyId(id) }

    pub(crate) fn index(&self) -> usize {
        self.0
    }
}