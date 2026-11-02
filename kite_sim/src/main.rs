pub mod recorder;

use glam::{DMat3, DQuat, DVec3};
use kite_core::dynamics::sources::gravity_source::ConstantGravity;
use kite_core::error::PhysicsError;
use kite_core::system::state::State;
use kite_core::world::world_builder::{BodyDesc, WorldBuilder};
use recorder::Recorder;

fn main() -> Result<(), Box<dyn std::error::Error>> {
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


    let mut rec = Recorder::new("output/run.csv", vec![body1])?;
    rec.record(0.0, &world)?;                 // initial state
    for step in 1..=2400 {
        world.step();
        if step % 4 == 0 {                    // record every 4th step
            rec.record(world.time.elapsed_seconds(), &world)?;
        }
    }
    rec.finish()?;

    Ok(())
}
