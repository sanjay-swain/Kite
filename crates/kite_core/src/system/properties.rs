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
            if *mass <= 0.0 || !mass.is_finite() {
                return Err(PhysicsError::InvalidMass(*mass))
            } else {
                masses_inverse.push(1.0/mass);
            }
        }

        // Validate the inertia matrix
        for inertia in &inertias {
            // 1. All diagonal elements must be greater than zero
            let diagonal = inertia.diagonal();
            if (diagonal.x <= 0.0 || !diagonal.x.is_finite())
                || (diagonal.y <= 0.0 || !diagonal.y.is_finite())
                || (diagonal.z <= 0.0 || !diagonal.z.is_finite()) {
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

    pub fn len(&self) -> usize { self.masses.len() }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyId(usize);

impl BodyId {
    pub(crate) fn new(id: usize) -> Self { BodyId(id) }

    pub(crate) fn index(&self) -> usize {
        self.0
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;

    const EPS: f64 = 1e-9;

    fn diag(a: f64, b: f64, c: f64) -> DMat3 {
        DMat3::from_diagonal(DVec3::new(a, b, c))
    }

    /// Symmetric, non-diagonal, positive definite. A transposed or
    /// misplaced element in the inverse would show up with this.
    fn general_inertia() -> DMat3 {
        DMat3::from_cols(
            DVec3::new(2.0, 0.1, 0.2),
            DVec3::new(0.1, 3.0, 0.3),
            DVec3::new(0.2, 0.3, 4.0),
        )
    }

    fn three_bodies() -> Properties {
        Properties::new(
            vec![1.0, 2.0, 4.0],
            vec![diag(1.0, 1.0, 1.0), diag(2.0, 3.0, 4.0), general_inertia()],
        )
            .unwrap()
    }

    // ---------- accessors return the right body ----------

    #[test]
    fn masses_are_returned_per_body() {
        let p = three_bodies();
        assert_eq!(p.mass(BodyId::new(0)), 1.0);
        assert_eq!(p.mass(BodyId::new(1)), 2.0);
        assert_eq!(p.mass(BodyId::new(2)), 4.0);
    }

    #[test]
    fn inertias_are_returned_per_body() {
        let p = three_bodies();
        assert!(p.inertia(BodyId::new(0)).abs_diff_eq(diag(1.0, 1.0, 1.0), EPS));
        assert!(p.inertia(BodyId::new(1)).abs_diff_eq(diag(2.0, 3.0, 4.0), EPS));
        assert!(p.inertia(BodyId::new(2)).abs_diff_eq(general_inertia(), EPS));
    }

    #[test]
    fn mass_inverse_is_returned_per_body() {
        let p = three_bodies();
        assert!((p.mass_inverse(BodyId::new(0)) - 1.0).abs() < EPS);
        assert!((p.mass_inverse(BodyId::new(1)) - 0.5).abs() < EPS);
        assert!((p.mass_inverse(BodyId::new(2)) - 0.25).abs() < EPS);
    }

    #[test]
    fn inertia_inverse_of_diagonal_is_reciprocal() {
        let p = three_bodies();
        let expected = diag(1.0 / 2.0, 1.0 / 3.0, 1.0 / 4.0);
        assert!(p.inertia_inverse(BodyId::new(1)).abs_diff_eq(expected, EPS));
    }

    // ---------- inverses are true inverses ----------

    #[test]
    fn mass_times_inverse_is_one() {
        let p = three_bodies();
        for i in 0..3 {
            let id = BodyId::new(i);
            assert!((p.mass(id) * p.mass_inverse(id) - 1.0).abs() < EPS);
        }
    }

    #[test]
    fn inertia_times_inverse_is_identity() {
        let p = three_bodies();
        for i in 0..3 {
            let id = BodyId::new(i);
            let product = p.inertia(id) * p.inertia_inverse(id);
            assert!(
                product.abs_diff_eq(DMat3::IDENTITY, EPS),
                "body {i}: {product:?}"
            );
        }
    }

    #[test]
    fn inverse_times_inertia_is_identity() {
        let p = three_bodies();
        let id = BodyId::new(2);
        let product = p.inertia_inverse(id) * p.inertia(id);
        assert!(product.abs_diff_eq(DMat3::IDENTITY, EPS));
    }

    #[test]
    fn single_body_works() {
        let p = Properties::new(vec![5.0], vec![DMat3::IDENTITY]).unwrap();
        assert_eq!(p.mass(BodyId::new(0)), 5.0);
        assert!((p.mass_inverse(BodyId::new(0)) - 0.2).abs() < EPS);
    }

    // ---------- invalid mass ----------

    #[test]
    fn zero_mass_is_rejected() {
        let r = Properties::new(vec![0.0], vec![DMat3::IDENTITY]);
        assert!(matches!(r, Err(PhysicsError::InvalidMass(_))));
    }

    #[test]
    fn negative_mass_is_rejected() {
        let r = Properties::new(vec![-1.0], vec![DMat3::IDENTITY]);
        assert!(matches!(r, Err(PhysicsError::InvalidMass(_))));
    }

    #[test]
    fn nan_mass_is_rejected() {
        let r = Properties::new(vec![f64::NAN], vec![DMat3::IDENTITY]);
        assert!(matches!(r, Err(PhysicsError::InvalidMass(_))));
    }

    #[test]
    fn infinite_mass_is_rejected() {
        let r = Properties::new(vec![f64::INFINITY], vec![DMat3::IDENTITY]);
        assert!(matches!(r, Err(PhysicsError::InvalidMass(_))));
    }

    #[test]
    fn one_bad_mass_among_good_ones_is_rejected() {
        let r = Properties::new(
            vec![1.0, -2.0, 3.0],
            vec![DMat3::IDENTITY, DMat3::IDENTITY, DMat3::IDENTITY],
        );
        assert!(matches!(r, Err(PhysicsError::InvalidMass(_))));
    }

    // ---------- invalid inertia ----------

    #[test]
    fn singular_inertia_is_rejected() {
        let r = Properties::new(vec![1.0], vec![diag(1.0, 0.0, 1.0)]);
        assert!(matches!(r, Err(PhysicsError::InvalidInertia(_))));
    }

    #[test]
    fn zero_matrix_inertia_is_rejected() {
        let r = Properties::new(vec![1.0], vec![DMat3::ZERO]);
        assert!(matches!(r, Err(PhysicsError::InvalidInertia(_))));
    }

    #[test]
    fn nan_inertia_is_rejected() {
        let r = Properties::new(vec![1.0], vec![diag(1.0, f64::NAN, 1.0)]);
        assert!(matches!(r, Err(PhysicsError::InvalidInertia(_))));
    }

    // ---------- bad input shapes ----------

    #[test]
    fn mismatched_lengths_are_rejected() {
        let r = Properties::new(vec![1.0, 2.0], vec![DMat3::IDENTITY]);
        assert!(r.is_err());

        let r = Properties::new(vec![1.0], vec![DMat3::IDENTITY, DMat3::IDENTITY]);
        assert!(r.is_err());
    }

    #[test]
    fn empty_input_is_allowed() {
        let p = Properties::new(vec![], vec![]).unwrap();
        assert_eq!(p.len(), 0);
    }
}