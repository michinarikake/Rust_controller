use crate::models::force::force_trait::Force;
use crate::models::state::state_trait::StateVector;

pub trait DisturbanceCalculator<T: StateVector, U: Force> {
    fn calc_force(&self, state: &T) -> U;
}