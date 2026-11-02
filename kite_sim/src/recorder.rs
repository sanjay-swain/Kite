use std::{fs::File, io::BufWriter, path::Path};

use serde::Serialize;
use kite_core::system::energy::kinetic_energy;
use kite_core::system::properties::BodyId;
use kite_core::system::state::State;
use kite_core::world::world::World;

#[derive(Serialize)]
struct Row {
    t: f64,
    body: usize,
    px: f64, py: f64, pz: f64,
    vx: f64, vy: f64, vz: f64,
    qx: f64, qy: f64, qz: f64, qw: f64,
    wx: f64, wy: f64, wz: f64,
    e: f64
}

impl Row {
    fn new(t: f64, body: usize, s: &State, e: f64) -> Self {
        Self {
            t,
            body,
            px: s.position.x, py: s.position.y, pz: s.position.z,
            vx: s.velocity.x, vy: s.velocity.y, vz: s.velocity.z,
            qx: s.orientation.x, qy: s.orientation.y,
            qz: s.orientation.z, qw: s.orientation.w,
            wx: s.angular_velocity.x, wy: s.angular_velocity.y, wz: s.angular_velocity.z,
            e
        }
    }
}

pub struct Recorder {
    writer: csv::Writer<BufWriter<File>>,
    ids: Vec<BodyId>,
}

impl Recorder {
    /// `ids` are the bodies to record, in the order you want them numbered.
    pub fn new(path: impl AsRef<Path>, ids: Vec<BodyId>) -> Result<Self, csv::Error> {
        let file = BufWriter::new(File::create(path)?);
        Ok(Self { writer: csv::Writer::from_writer(file), ids })
    }

    /// Writes one row per tracked body at time `t`.
    pub fn record(&mut self, t: f64, world: &World) -> Result<(), csv::Error> {
        for (i, id) in self.ids.iter().enumerate() {
            self.writer.serialize(Row::new(t, i, &world.state(*id), kinetic_energy(world.states(), &world.properties, *id)))?;
        }
        Ok(())
    }

    pub fn finish(mut self) -> Result<(), csv::Error> {
        self.writer.flush()?;
        Ok(())
    }
}