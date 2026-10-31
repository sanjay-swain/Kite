use crate::constraint::solvers::{ConstraintSolver, ConstraintSolverKind};
use crate::dynamics::sources::ForceSource;
use crate::dynamics::{Dynamics, DynamicsKind};
use crate::error::PhysicsError;
use crate::integrator::{Integrator, IntegratorKind};
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
    // pub fn set_dynamics_solver(&mut self) -> Box<dyn Dynamics> {
    //     self.dynamics.create(self.bodies.len())
    // }

    pub fn build(self) -> Result<World, PhysicsError> {
        let n = self.bodies.len();

        let mut states: Vec<State> = Vec::new();
        let mut masses: Vec<f64> = Vec::new();
        let mut inertias: Vec<DMat3> = Vec::new();
        for body in self.bodies {
            states.push(body.state);
            masses.push(body.mass);
            inertias.push(body.inertia);
        }

        let properties = Properties::new(masses, inertias)?;

        let dynamics = self.dynamics.create(n);
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