use csv::Writer;
use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::AccelerationConstraint,
        forces::DynamicSolver,
        gravity::{BasicEarthGravity, GravitySolver},
        newton_euler::NewtonEuler,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    system::{state::State, world::World},
};
use serde::Serialize;
use std::error::Error;

#[derive(Serialize)]
struct Telemetry {
    time_sec: f64,
    pos_x: f64,
    pos_y: f64,
    pos_z: f64,
    velocity_mag: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    println!("Starting");
    let mut wtr = Writer::from_path("orbital_telemetry.csv")?;
    // Simulator Config
    let dynamic_solver = NewtonEuler {};
    let constraint_solver = AccelerationConstraint {};
    let integrator = {
        let this = SemiImplicitEuler::new(1e-3);
        match this {
            Ok(t) => t,
            Err(_) => panic!("called `Result::unwrap()` on an `Err` value"),
        }
    };
    let gravity = BasicEarthGravity {};

    // Initialize world
    let mut world: World<
        NewtonEuler,
        AccelerationConstraint,
        SemiImplicitEuler,
        BasicEarthGravity,
    > = World::new(dynamic_solver, constraint_solver, integrator, gravity);

    match world.create_body(
        3.0,
        DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0)),
        State {
            position: DVec3::new(6771000.0, 0.0, 0.0),
            velocity: DVec3::new(0.0, 7672.6, 0.0),
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let mut t: f64 = 0.0;

    let mut i = 0;

    while t < 86400.0 {
        world.gravity_solver.solve(&mut world.bodies);
        // Update the state_derivative of each body
        world.dynamic_solver.solve(&mut world.bodies);

        // increase the time
        world.integrator.step(&mut world.bodies);

        t += world.integrator.step_size;

        if i % 100000 == 0 {
            wtr.serialize(Telemetry {
                time_sec: t,
                pos_x: world.bodies[0].state.position.x,
                pos_y: world.bodies[0].state.position.y,
                pos_z: world.bodies[0].state.position.z,
                velocity_mag: world.bodies[0].state.velocity.length(),
            })?;
        }

        // Clear all the forces at the end
        world.clear_forces_and_torques();
        i = i + 1;
    }
    println!("Finished");
    wtr.flush()?;

    println!("Telemetry saved successfully.");
    Ok(())
}
