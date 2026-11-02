use crate::system::properties::{BodyId, Properties};
use crate::system::state::State;

pub fn kinetic_energy(states: &[State], properties: &Properties, id: BodyId) -> f64 {
    0.5 * properties.mass(id) * (states[id.index()].velocity.dot(states[id.index()].velocity)) +
        0.5 * states[id.index()].angular_velocity.dot(properties.inertia(id).mul_vec3(states[id.index()].angular_velocity))

}