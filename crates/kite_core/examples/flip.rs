use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::AccelerationConstraint, forces::DynamicSolver, gravity::ConstantGravity,
        newton_euler::NewtonEuler,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    system::{state::State, world::World},
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

    match world.create_body(
        3.0,
        DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0)),
        State {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::new(0.0, 10.0, 0.01),
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let mut t: f64 = 0.0;

    while t < 10.0 {
        // Update the state_derivative of each body
        world.dynamic_solver.solve(&mut world.bodies);

        // increase the time
        world.integrator.step(&mut world.bodies);

        t += world.integrator.step_size;

        // Clear all the forces at the end
        world.clear_forces_and_torques();
    }
    println!("Finished");
    println!(
        "{}",
        (world.bodies[0].inertia * world.bodies[0].state.angular_velocity).length()
    );
}
