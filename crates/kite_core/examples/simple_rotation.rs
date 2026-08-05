use glam::{DMat3, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::AccelerationConstraint,
        forces::DynamicSolver,
        gravity::{ConstantGravity, GravitySolver},
        newton_euler::NewtonEuler,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    system::{
        interactions::{Frame, Torque},
        state::State,
        world::World,
    },
};

fn main() {
    println!("Starting");
    let force_solver = NewtonEuler {};
    let constraint_solver = AccelerationConstraint {};
    let integration = {
        let this = SemiImplicitEuler::new(1e-3);
        match this {
            Ok(t) => t,
            Err(_) => panic!("called `Result::unwrap()` on an `Err` value"),
        }
    };
    let gravity = ConstantGravity {
        field: DVec3 {
            x: 0.0,
            y: 0.0,
            z: -9.81,
        },
    };
    let mut world = World::new(force_solver, constraint_solver, integration, gravity);

    let _ = match world.create_body(3.0, DMat3::from_diagonal(DVec3::ONE), State::ZERO, false) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let mut t: f64 = 0.0;

    while t < 5.0 {
        // Apply forces
        world.gravity_solver.solve(&mut world.bodies);
        world.bodies[0].apply_torque(Torque::new(
            DVec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Frame::Local,
        ));

        // Update the state_derivative of each body
        world.dynamic_solver.solve(&mut world.bodies);

        // increase the time
        world.integrator.step(&mut world.bodies);

        t += world.integrator.step_size;

        // Clear all the forces at the end
        world.clear_forces_and_torques();
    }
    let (axis, angle) = world.bodies[0].state.orientation.to_axis_angle();
    println!("{} {}", axis, angle);
    println!("Finished");
}
