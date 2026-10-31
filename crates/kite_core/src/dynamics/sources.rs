use crate::{
    system::{
        loads::Load,
        properties::Properties,
        state::State
    }
};

pub mod gravity_source;

pub trait ForceSource {
    fn compute_loads(&self, properties: &Properties, states: &[State], t: f64, loads: &mut [Load]);
}