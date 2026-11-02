use crate::constraint::solvers::ConstraintSolverKind;
use crate::dynamics::sources::ForceSource;
use crate::dynamics::DynamicsKind;
use crate::error::PhysicsError;
use crate::integrator::IntegratorKind;
use crate::system::properties::{BodyId, Properties};
use crate::system::state::State;
use crate::system::time::SimTime;
use crate::world::world::World;
use glam::DMat3;

pub struct BodyDesc {
    pub mass: f64,
    pub inertia: DMat3,      // body frame
    pub state: State,
}

pub struct WorldBuilder {
    time: SimTime,
    bodies: Vec<BodyDesc>,
    sources: Vec<Box<dyn ForceSource>>,
    dynamics: DynamicsKind,
    integrator: IntegratorKind,
    constraint_solver: ConstraintSolverKind,
}

impl WorldBuilder {
    pub fn new(hz: u64) -> Result<Self, PhysicsError> {
        Ok(Self {
            time: SimTime::new(hz)?,
            bodies: Vec::new(),
            sources: Vec::new(),
            dynamics: DynamicsKind::NewtonEuler,
            integrator: IntegratorKind::SemiImplicitEuler,
            constraint_solver: ConstraintSolverKind::NoSolver
        })
    }

    pub fn add_body(&mut self, desc: BodyDesc) -> BodyId {
        self.bodies.push(desc);
        BodyId::new(self.bodies.len() - 1)
    }

    pub fn add_force_source(&mut self, s: Box<dyn ForceSource>) {
        self.sources.push(s);
    }

    // Maybe we do not need these?
    // pub fn set_integrator(&mut self) -> Box<dyn Integrator> {
    //     self.integrator.create(self.bodies.len())
    // }
    //
    // pub fn set_constraint_solver(&mut self) -> Box<dyn ConstraintSolver> {
    //     self.constraint_solver.create(self.bodies.len())
    // }
    //

    pub fn build(self) -> Result<World, PhysicsError> {
        let n = self.bodies.len();

        let mut states: Vec<State> = Vec::new();
        let mut masses: Vec<f64> = Vec::new();
        let mut inertias: Vec<DMat3> = Vec::new();
        for body in self.bodies {
            if body.state.is_valid() {
                states.push(body.state);
                masses.push(body.mass);
                inertias.push(body.inertia);
            }
            else {
                return Err(PhysicsError::InvalidState)
            }
        }

        let properties = Properties::new(masses, inertias)?;

        let mut dynamics = self.dynamics.create(n);
        dynamics.set_force_sources(self.sources);

        let integrator = self.integrator.create(n);
        let constraint_solver = self.constraint_solver.create(n);

        Ok(World::new(
            self.time,
            states,
            properties,
            Vec::new(),
            dynamics,
            constraint_solver,
            integrator
        ))
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::sources::gravity_source::ConstantGravity;
    use glam::{DQuat, DVec3};

    const EPS: f64 = 1e-9;

    fn state_at(pos: DVec3, vel: DVec3) -> State {
        State {
            position: pos,
            velocity: vel,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        }
    }

    fn desc(mass: f64) -> BodyDesc {
        BodyDesc {
            mass,
            inertia: DMat3::IDENTITY,
            state: state_at(DVec3::ZERO, DVec3::ZERO),
        }
    }

    // ---------- construction ----------

    #[test]
    fn zero_hz_is_rejected() {
        assert!(matches!(
            WorldBuilder::new(0),
            Err(PhysicsError::InvalidFrequency(0))
        ));
    }

    #[test]
    fn valid_hz_is_accepted() {
        assert!(WorldBuilder::new(240).is_ok());
    }

    // ---------- ids ----------

    #[test]
    fn ids_are_sequential_from_zero() {
        let mut b = WorldBuilder::new(60).unwrap();
        assert_eq!(b.add_body(desc(1.0)), BodyId::new(0));
        assert_eq!(b.add_body(desc(1.0)), BodyId::new(1));
        assert_eq!(b.add_body(desc(1.0)), BodyId::new(2));
    }

    #[test]
    fn identical_descriptions_still_get_distinct_ids() {
        let mut b = WorldBuilder::new(60).unwrap();
        let a = b.add_body(desc(1.0));
        let c = b.add_body(desc(1.0));
        assert_ne!(a, c);
    }

    #[test]
    fn each_id_reads_back_its_own_initial_state() {
        let mut b = WorldBuilder::new(60).unwrap();
        let ids: Vec<_> = (0..3)
            .map(|i| {
                let x = i as f64 + 1.0;
                b.add_body(BodyDesc {
                    mass: x,
                    inertia: DMat3::IDENTITY,
                    state: state_at(DVec3::new(x, 0.0, 0.0), DVec3::ZERO),
                })
            })
            .collect();
        let w = b.build().unwrap();
        for (i, id) in ids.iter().enumerate() {
            let expected = DVec3::new(i as f64 + 1.0, 0.0, 0.0);
            assert!(w.state(*id).position.abs_diff_eq(expected, EPS));
        }
    }

    // ---------- validation in build() ----------

    #[test]
    fn bad_mass_is_rejected_at_build() {
        for m in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut b = WorldBuilder::new(60).unwrap();
            b.add_body(desc(m));
            assert!(
                matches!(b.build(), Err(PhysicsError::InvalidMass(_))),
                "mass {m}"
            );
        }
    }

    #[test]
    fn one_bad_body_among_good_ones_fails_the_build() {
        let mut b = WorldBuilder::new(60).unwrap();
        b.add_body(desc(1.0));
        b.add_body(desc(-2.0));
        b.add_body(desc(3.0));
        assert!(b.build().is_err());
    }

    #[test]
    fn singular_inertia_is_rejected_at_build() {
        let mut b = WorldBuilder::new(60).unwrap();
        b.add_body(BodyDesc {
            mass: 1.0,
            inertia: DMat3::ZERO,
            state: state_at(DVec3::ZERO, DVec3::ZERO),
        });
        assert!(matches!(b.build(), Err(PhysicsError::InvalidInertia(_))));
    }

    #[test]
    fn non_finite_initial_state_is_rejected() {
        let mut b = WorldBuilder::new(60).unwrap();
        b.add_body(BodyDesc {
            mass: 1.0,
            inertia: DMat3::IDENTITY,
            state: state_at(DVec3::new(f64::NAN, 0.0, 0.0), DVec3::ZERO),
        });
        assert!(b.build().is_err());
    }

    #[test]
    fn empty_world_is_allowed() { assert!(WorldBuilder::new(60).unwrap().build().is_ok()); }

    // ---------- configuration reaches the world ----------

    #[test]
    fn no_sources_means_straight_line_motion_at_the_chosen_hz() {
        // 10 Hz, 10 steps = exactly 1 s, so position = velocity * 1.
        let mut b = WorldBuilder::new(10).unwrap();
        let id = b.add_body(BodyDesc {
            mass: 1.0,
            inertia: DMat3::IDENTITY,
            state: state_at(DVec3::ZERO, DVec3::new(1.0, 2.0, 3.0)),
        });
        let mut w = b.build().unwrap();
        for _ in 0..10 {
            w.step();
        }
        assert!(w.state(id).position.abs_diff_eq(DVec3::new(1.0, 2.0, 3.0), 1e-9));
        assert!(w.state(id).velocity.abs_diff_eq(DVec3::new(1.0, 2.0, 3.0), EPS));
    }

    #[test]
    fn hz_controls_the_step_size() {
        // One step at 100 Hz moves a 1 m/s body by 0.01 m.
        let mut b = WorldBuilder::new(100).unwrap();
        let id = b.add_body(BodyDesc {
            mass: 1.0,
            inertia: DMat3::IDENTITY,
            state: state_at(DVec3::ZERO, DVec3::X),
        });
        let mut w = b.build().unwrap();
        w.step();
        assert!((w.state(id).position.x - 0.01).abs() < EPS);
    }

    #[test]
    fn registered_source_reaches_the_dynamics() {
        // After one step at 100 Hz, v = g * dt for every Euler-type scheme.
        let g = DVec3::new(0.0, -9.81, 0.0);
        let mut b = WorldBuilder::new(100).unwrap();
        let light = b.add_body(desc(1.0));
        let heavy = b.add_body(desc(50.0));
        b.add_force_source(Box::new(ConstantGravity { field: g }));
        let mut w = b.build().unwrap();
        w.step();
        let expected = g * 0.01;
        assert!(w.state(light).velocity.abs_diff_eq(expected, EPS));
        assert!(w.state(heavy).velocity.abs_diff_eq(expected, EPS));
    }

    #[test]
    fn multiple_sources_all_reach_the_dynamics() {
        let mut b = WorldBuilder::new(100).unwrap();
        let id = b.add_body(desc(1.0));
        b.add_force_source(Box::new(ConstantGravity { field: DVec3::X }));
        b.add_force_source(Box::new(ConstantGravity { field: DVec3::Y }));
        let mut w = b.build().unwrap();
        w.step();
        assert!(w.state(id).velocity.abs_diff_eq(DVec3::new(0.01, 0.01, 0.0), EPS));
    }

    // ---------- sizes ----------

    #[test]
    fn multi_body_world_steps_without_panicking() {
        // Exercises every scratch buffer (loads, derivatives, stage states) at n = 5.
        let mut b = WorldBuilder::new(120).unwrap();
        for i in 0..5 {
            b.add_body(BodyDesc {
                mass: 1.0 + i as f64,
                inertia: DMat3::IDENTITY,
                state: state_at(DVec3::new(i as f64, 0.0, 0.0), DVec3::Y),
            });
        }
        b.add_force_source(Box::new(ConstantGravity { field: DVec3::new(0.0, -9.81, 0.0) }));
        let mut w = b.build().unwrap();
        for _ in 0..200 {
            w.step();
        }
    }

    #[test]
    fn bodies_do_not_influence_each_other_without_interaction() {
        // Body 0 moves, body 1 sits still; stepping must not leak between them.
        let mut b = WorldBuilder::new(60).unwrap();
        let moving = b.add_body(BodyDesc {
            mass: 1.0,
            inertia: DMat3::IDENTITY,
            state: state_at(DVec3::ZERO, DVec3::X),
        });
        let still = b.add_body(desc(1.0));
        let mut w = b.build().unwrap();
        for _ in 0..60 {
            w.step();
        }
        assert!(w.state(still).position.abs_diff_eq(DVec3::ZERO, EPS));
        assert!(w.state(moving).position.abs_diff_eq(DVec3::X, 1e-9));
    }

    // ---------- rotation passes through ----------

    #[test]
    fn initial_angular_velocity_rotates_the_body() {
        // 1 rad/s about Z for 1 s: z = sin(0.5), w = cos(0.5).
        let mut b = WorldBuilder::new(240).unwrap();
        let id = b.add_body(BodyDesc {
            mass: 1.0,
            inertia: DMat3::IDENTITY,
            state: State {
                position: DVec3::ZERO,
                velocity: DVec3::ZERO,
                orientation: DQuat::IDENTITY,
                angular_velocity: DVec3::Z,
            },
        });
        let mut w = b.build().unwrap();
        for _ in 0..240 {
            w.step();
        }
        let q = w.state(id).orientation;
        assert!((q.z - 0.5_f64.sin()).abs() < 1e-4);
        assert!((q.w - 0.5_f64.cos()).abs() < 1e-4);
    }
}