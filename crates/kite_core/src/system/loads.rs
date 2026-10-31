use glam::{DQuat, DVec3};

/// Specifies the reference frame of a vector
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Frame {
    /// The vector is defined relative to the world's origin
    World,
    /// The vector is defined relative to the body center of mass.
    Body,
}

/// This struct holds information of the force acting on a body.
/// Unit: Newton(N)
#[derive(Clone, Copy)]
pub struct Force {
    pub vector: DVec3,
    /// The point where force is applied.
    /// This is ALWAYS with respect to body frame of reference
    /// regardless the frame assigned is Body or World
    pub point: DVec3,
    /// This defines whether the force is being applied is oriented according to
    /// Body frame of reference or World
    pub frame: Frame,
}

impl Force {
    /// Create a new force
    pub fn new(force: DVec3, position: DVec3, frame: Frame) -> Self {
        Self {
            vector: force,
            point: position,
            frame,
        }
    }

    /// Return the force vector in world frame
    /// If the force is already in world frame it will return the same vector
    pub fn to_world(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => self.vector,
            Frame::Body => orientation * self.vector,
        }
    }

    /// Return the force vector in body frame
    /// If the force is already in body frame it will return the same vector
    pub fn to_body(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => orientation.inverse() * self.vector,
            Frame::Body => self.vector,
        }
    }
}

/// The struct holds information of pure torque applied on a body
/// Unit: Newton-meter (Nm)
#[derive(Clone, Copy)]
pub struct Torque {
    /// The torque applied in Nm
    pub torque: DVec3,
    /// Frame of reference about which the torque is being applied
    pub frame: Frame,
}

impl Torque {
    /// Creates a new Torque
    pub fn new(torque: DVec3, frame: Frame) -> Self {
        Self {
            torque,
            frame,
        }
    }

    /// Return the torque vector in world frame of reference
    /// If the torque is already in world frame of reference then return the same.
    pub fn to_world(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => self.torque,
            Frame::Body => orientation * self.torque,
        }
    }

    /// Return the torque vector in body frame of reference
    /// If the torque is already in body frame of reference then return the same
    pub fn to_body(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::World => orientation.inverse() * self.torque,
            Frame::Body => self.torque,
        }
    }
}

/// Net load on a body
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Load {
    /// Net force, world frame
    force: DVec3,
    /// Net torque about the center of mass, world frame
    torque: DVec3,
}


impl Load {
    pub fn force(&self) -> DVec3 { self.force }
    pub fn torque(&self) -> DVec3 { self.torque }

    /// Adds a force (and the torque it creates about the center of mass).
    pub fn add_force(&mut self, f: Force, orientation: DQuat) {
        self.force += f.to_world(orientation);
        self.torque += (orientation * f.point).cross(f.to_world(orientation));
    }

    /// Adds a pure torque.
    pub fn add_torque(&mut self, t: Torque, orientation: DQuat) {
        self.torque += t.to_world(orientation);
    }

    pub fn reset(&mut self) {
        self.force = DVec3::ZERO;
        self.torque = DVec3::ZERO;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::EulerRot;
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-9;

    fn rot_x90() -> DQuat {
        DQuat::from_axis_angle(DVec3::X, FRAC_PI_2)
    }

    fn rot_z90() -> DQuat {
        DQuat::from_axis_angle(DVec3::Z, FRAC_PI_2)
    }

    /// Orientation with unequal angles on all axes, so swapped axes can't hide.
    fn rot_general() -> DQuat {
        DQuat::from_euler(EulerRot::XYZ, 0.3, -0.7, 1.1)
    }

    fn assert_vec_eq(actual: DVec3, expected: DVec3) {
        assert!(
            actual.abs_diff_eq(expected, EPS),
            "expected {expected:?}, got {actual:?}"
        );
    }

    // ---------- Force / Torque frame conversion ----------

    #[test]
    fn identity_orientation_changes_nothing() {
        let v = DVec3::new(10.0, 20.0, 30.0);
        let f = Force::new(v, DVec3::ZERO, Frame::Body);
        let tau = Torque::new(v, Frame::Body);

        assert_vec_eq(f.to_world(DQuat::IDENTITY), v);
        assert_vec_eq(tau.to_world(DQuat::IDENTITY), v);
    }

    #[test]
    fn body_to_world_rotates_about_x() {
        // +90 deg about X sends (x, y, z) -> (x, -z, y)
        let v = DVec3::new(10.0, 20.0, 30.0);
        let expected = DVec3::new(10.0, -30.0, 20.0);

        let f = Force::new(v, DVec3::ZERO, Frame::Body);
        let tau = Torque::new(v, Frame::Body);

        assert_vec_eq(f.to_world(rot_x90()), expected);
        assert_vec_eq(tau.to_world(rot_x90()), expected);
    }

    #[test]
    fn world_to_body_is_inverse_rotation_about_x() {
        let v = DVec3::new(10.0, -30.0, 20.0);
        let expected = DVec3::new(10.0, 20.0, 30.0);

        let f = Force::new(v, DVec3::ZERO, Frame::World);
        let tau = Torque::new(v, Frame::World);

        assert_vec_eq(f.to_body(rot_x90()), expected);
        assert_vec_eq(tau.to_body(rot_x90()), expected);
    }

    #[test]
    fn world_frame_vector_is_unchanged_by_to_world() {
        let v = DVec3::new(1.0, 2.0, 3.0);
        let f = Force::new(v, DVec3::ZERO, Frame::World);
        let tau = Torque::new(v, Frame::World);

        assert_vec_eq(f.to_world(rot_general()), v);
        assert_vec_eq(tau.to_world(rot_general()), v);
    }

    #[test]
    fn body_frame_vector_is_unchanged_by_to_body() {
        let v = DVec3::new(1.0, 2.0, 3.0);
        let f = Force::new(v, DVec3::ZERO, Frame::Body);
        let tau = Torque::new(v, Frame::Body);

        assert_vec_eq(f.to_body(rot_general()), v);
        assert_vec_eq(tau.to_body(rot_general()), v);
    }

    #[test]
    fn body_to_world_to_body_round_trips() {
        let v = DVec3::new(1.0, -2.0, 3.5);
        let q = rot_general();

        // Body-frame vector -> world.
        let world = Force::new(v, DVec3::ZERO, Frame::Body).to_world(q);
        // Treat that result as a world-frame vector and bring it back.
        let back = Force::new(world, DVec3::ZERO, Frame::World).to_body(q);
        assert_vec_eq(back, v);

        let world_t = Torque::new(v, Frame::Body).to_world(q);
        let back_t = Torque::new(world_t, Frame::World).to_body(q);
        assert_vec_eq(back_t, v);
    }

    #[test]
    fn rotation_preserves_vector_length() {
        let v = DVec3::new(3.0, 4.0, 12.0); // length 13
        let w = Force::new(v, DVec3::ZERO, Frame::Body).to_world(rot_general());
        assert!((w.length() - 13.0).abs() < EPS);
    }

    #[test]
    fn default_load_is_zero() {
        let load = Load::default();
        assert_eq!(load.force(), DVec3::ZERO);
        assert_eq!(load.torque(), DVec3::ZERO);
    }

    // ---------- Load::add_force ----------

    #[test]
    fn force_at_center_of_mass_adds_no_torque() {
        let mut load = Load::default();
        let f = Force::new(DVec3::new(1.0, 2.0, 3.0), DVec3::ZERO, Frame::World);
        load.add_force(f, rot_general());

        assert_vec_eq(load.force(), DVec3::new(1.0, 2.0, 3.0));
        assert_vec_eq(load.torque(), DVec3::ZERO);
    }

    #[test]
    fn force_along_lever_arm_adds_no_torque() {
        // Force parallel to the offset: r x F = 0.
        let mut load = Load::default();
        let f = Force::new(
            DVec3::new(5.0, 0.0, 0.0),
            DVec3::new(2.0, 0.0, 0.0),
            Frame::World,
        );
        load.add_force(f, DQuat::IDENTITY);

        assert_vec_eq(load.force(), DVec3::new(5.0, 0.0, 0.0));
        assert_vec_eq(load.torque(), DVec3::ZERO);
    }

    #[test]
    fn offset_force_creates_torque_identity_orientation() {
        // r = (1,0,0), F = (0,1,0)  =>  r x F = (0,0,1)
        let mut load = Load::default();
        let f = Force::new(
            DVec3::new(0.0, 1.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
            Frame::World,
        );
        load.add_force(f, DQuat::IDENTITY);

        assert_vec_eq(load.force(), DVec3::new(0.0, 1.0, 0.0));
        assert_vec_eq(load.torque(), DVec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn lever_arm_rotates_with_body_world_force() {
        // Body rotated +90 deg about Z: body point (1,0,0) -> world (0,1,0).
        // World force (1,0,0). r x F = (0,1,0) x (1,0,0) = (0,0,-1).
        // A wrong rotation direction would give (0,0,+1).
        let mut load = Load::default();
        let f = Force::new(
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
            Frame::World,
        );
        load.add_force(f, rot_z90());

        assert_vec_eq(load.force(), DVec3::new(1.0, 0.0, 0.0));
        assert_vec_eq(load.torque(), DVec3::new(0.0, 0.0, -1.0));
    }

    #[test]
    fn body_fixed_force_gives_same_torque_regardless_of_orientation() {
        // Force and point both fixed to the body: torque expressed in world
        // is the body-frame torque rotated. Here body torque is
        // (1,0,0) x (0,1,0) = (0,0,1), and rotating about Z leaves it alone.
        let f = Force::new(
            DVec3::new(0.0, 1.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
            Frame::Body,
        );

        let mut a = Load::default();
        a.add_force(f, DQuat::IDENTITY);
        assert_vec_eq(a.torque(), DVec3::new(0.0, 0.0, 1.0));

        let mut b = Load::default();
        b.add_force(f, rot_z90());
        // World force is (-1,0,0), world lever arm is (0,1,0):
        // (0,1,0) x (-1,0,0) = (0,0,1)
        assert_vec_eq(b.force(), DVec3::new(-1.0, 0.0, 0.0));
        assert_vec_eq(b.torque(), DVec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn body_fixed_torque_equals_rotated_body_torque_general_orientation() {
        // For a fully body-fixed force, world torque must equal
        // q * (r_body x F_body). Checked at an awkward orientation.
        let q = rot_general();
        let r = DVec3::new(0.5, -1.0, 2.0);
        let force = DVec3::new(3.0, 1.0, -2.0);

        let mut load = Load::default();
        load.add_force(Force::new(force, r, Frame::Body), q);

        assert_vec_eq(load.torque(), q * r.cross(force));
        assert_vec_eq(load.force(), q * force);
    }

    #[test]
    fn forces_accumulate() {
        let mut load = Load::default();
        load.add_force(
            Force::new(DVec3::new(1.0, 2.0, 3.0), DVec3::ZERO, Frame::World),
            DQuat::IDENTITY,
        );
        load.add_force(
            Force::new(DVec3::new(4.0, 5.0, 6.0), DVec3::ZERO, Frame::World),
            DQuat::IDENTITY,
        );

        assert_vec_eq(load.force(), DVec3::new(5.0, 7.0, 9.0));
    }

    #[test]
    fn opposite_forces_at_offsets_form_a_couple() {
        // Equal and opposite forces: net force 0, net torque nonzero.
        // +Y at (1,0,0) gives (0,0,1); -Y at (-1,0,0) gives (-1,0,0)x(0,-1,0) = (0,0,1).
        let mut load = Load::default();
        load.add_force(
            Force::new(DVec3::Y, DVec3::X, Frame::World),
            DQuat::IDENTITY,
        );
        load.add_force(
            Force::new(-DVec3::Y, -DVec3::X, Frame::World),
            DQuat::IDENTITY,
        );

        assert_vec_eq(load.force(), DVec3::ZERO);
        assert_vec_eq(load.torque(), DVec3::new(0.0, 0.0, 2.0));
    }

    // ---------- Load::add_torque ----------

    #[test]
    fn pure_torque_does_not_change_force() {
        let mut load = Load::default();
        load.add_torque(
            Torque::new(DVec3::new(1.0, 2.0, 3.0), Frame::World),
            rot_general(),
        );

        assert_vec_eq(load.force(), DVec3::ZERO);
        assert_vec_eq(load.torque(), DVec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn body_frame_torque_is_rotated_to_world() {
        // +90 deg about Z sends (1,0,0) -> (0,1,0)
        let mut load = Load::default();
        load.add_torque(Torque::new(DVec3::X, Frame::Body), rot_z90());

        assert_vec_eq(load.torque(), DVec3::Y);
    }

    #[test]
    fn torques_accumulate() {
        let mut load = Load::default();
        load.add_torque(Torque::new(DVec3::new(1.0, 0.0, 0.0), Frame::World), DQuat::IDENTITY);
        load.add_torque(Torque::new(DVec3::new(0.0, 2.0, 0.0), Frame::World), DQuat::IDENTITY);

        assert_vec_eq(load.torque(), DVec3::new(1.0, 2.0, 0.0));
    }

    #[test]
    fn force_torque_and_pure_torque_combine() {
        // Offset force gives (0,0,1); pure torque adds (1,0,0).
        let mut load = Load::default();
        load.add_force(
            Force::new(DVec3::Y, DVec3::X, Frame::World),
            DQuat::IDENTITY,
        );
        load.add_torque(Torque::new(DVec3::X, Frame::World), DQuat::IDENTITY);

        assert_vec_eq(load.force(), DVec3::Y);
        assert_vec_eq(load.torque(), DVec3::new(1.0, 0.0, 1.0));
    }
}