use crate::{
    constraint::{
        constraint::Constraint,
        solvers::ConstraintSolver,
    },
    dynamics::Dynamics,
    integrator::Integrator,
    system::{
        properties::BodyId,
        properties::Properties,
        state::State,
        time::SimTime
    },
};

pub struct World
{
    pub time: SimTime,
    states: Vec<State>,

    pub properties: Properties,
    pub constraints: Vec<Constraint>,

    pub dynamics: Box<dyn Dynamics>,
    pub constraint_solver: Box<dyn ConstraintSolver>,
    pub integrator: Box<dyn Integrator>,
}

impl World {
    pub fn states(&self) -> &[State] {
        &self.states
    }

    pub fn state(&self, id: BodyId) -> State {
        self.states[id.index()]
    }

    pub fn new(
        time: SimTime,
        states: Vec<State>,
        properties: Properties,
        constraints: Vec<Constraint>,
        dynamics: Box<dyn Dynamics>,
        constraint_solver: Box<dyn ConstraintSolver>,
        integrator: Box<dyn Integrator>
    ) -> Self {
        Self {
            time,
            states,
            properties,
            constraints,
            constraint_solver,
            integrator,
            dynamics,
        }
    }

    pub fn step(&mut self) {
        let t = self.time.elapsed_seconds();
        let dt = self.time.dt_seconds();

        self.integrator.step(&mut self.dynamics, &self.properties, &mut self.states, t, dt);

        self.time.advance();
    }
}

