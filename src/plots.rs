use serde::Serialize;

#[derive(Serialize)]
pub struct PhysicsLog {
    pub time: f64,
    pub pos_x: f64,
    pub pos_y: f64,
    pub pos_z: f64,

    pub vel_x: f64,
    pub vel_y: f64,
    pub vel_z: f64,

    pub constraint_error: f64, // Important for checking solver health

    pub force_x: f64,
    pub force_y: f64,
    pub force_z: f64,
}
