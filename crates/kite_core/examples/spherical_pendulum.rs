use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::{AccelerationConstraint, ConstraintSolver},
        forces::DynamicSolver,
        gravity::{ConstantGravity, GravitySolver},
        newton_euler::NewtonEuler,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    system::{
        constraints::{joints::JointType, spherical::SphericalJoint},
        interactions::Force,
        state::State,
        world::{SimTime, World},
    },
};

fn main() {
    println!("Starting");
    let force_solver = NewtonEuler {};
    let constraint_solver = AccelerationConstraint {};
    let time_setup = {
        let this = SimTime::new(1000);
        match this {
            Ok(t) => t,
            Err(_) => panic!("The time step can't be zero"),
        }
    };
    let integration = SemiImplicitEuler::new(1e-3);
    let gravity = ConstantGravity {
        field: DVec3 {
            x: 0.0,
            y: 0.0,
            z: -9.81,
        },
    };
    let mut world = World::new(
        force_solver,
        constraint_solver,
        integration,
        gravity,
        time_setup,
    );

    let gr = match world.add_ground() {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let b1 = match world.create_body(
        1.0,
        DMat3::from_diagonal(DVec3::new(0.004, 0.004, 0.004)),
        State {
            position: DVec3::new(0.0, 0.0, 0.0),
            velocity: DVec3::new(0.0, 0.0, 0.0),
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::new(0.0, 0.0, 0.0),
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let spherical_joint = SphericalJoint::new();

    world.create_constraint(
        gr,
        b1,
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(0.0, 0.0, 1.0),
        DQuat::IDENTITY,
        DQuat::IDENTITY,
        JointType::SphericalJoint(spherical_joint),
    );

    let mut t: f64 = 0.0;

    while t < 10.0 {
        world.gravity_solver.solve(&mut world.bodies);

        world.bodies[b1].apply_force(Force::new(
            DVec3::X,
            DVec3::ZERO,
            kite_core::system::interactions::Frame::Local,
        ));

        for constraint in &mut world.constraints {
            world.constraint_solver.solve(constraint, &world.bodies);
        }

        world.apply_constraint_forces();

        world.dynamic_solver.solve(&mut world.bodies);

        world.integrator.step(&mut world.bodies);

        t += world.integrator.step_size;

        world.clear_forces_and_torques();
    }
    println!("{}", world.bodies[1].state.position);
}
