use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;

pub trait Cost<T: StateVector, U: Force> {
    fn stage_cost(&self, x: &T, v: &U) -> f64;
    fn terminal_cost(&self, x: &T) -> f64;
}
