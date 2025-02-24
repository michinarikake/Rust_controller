use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;

pub trait DisturbanceCalculator<T: StateVector, U: Force> {
    fn calc_force(&self, state: &T) -> U;
}