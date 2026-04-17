use serde::Serialize;

#[derive(Serialize)]
pub struct PhysicsLog {
    pub time: f64,
    pub pos_x: f64,
    pub pos_y: f64,
    pub pos_z: f64,
    pub vel_mag: f64,
    pub constraint_error: f64, // Important for checking solver health
    pub force_mag: f64,        // To see if impulses are exploding
}
