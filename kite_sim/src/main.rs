use glam::{DMat3, DQuat, DVec3};
use kite_core::dynamics::sources::gravity_source::ConstantGravity;
use kite_core::error::PhysicsError;
use kite_core::system::state::State;
use kite_core::world::world_builder::{BodyDesc, WorldBuilder};

fn main() -> Result<(), PhysicsError> {
    let mut world_builder = WorldBuilder::new(240)?;

    let body1 = world_builder.add_body(BodyDesc {
        mass: 1.0,
        inertia: DMat3::IDENTITY,
        state: State {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        }
    });

    world_builder.add_force_source(Box::new(ConstantGravity::new(DVec3::Z)));

    let mut world = world_builder.build()?;

    let max_time = 10.0;

    while world.time.elapsed_seconds() < max_time {
        world.step();
    }

    println!("Time: {} And state: \n{}", world.time.elapsed_seconds(), world.state(body1));

    Ok(())
}
