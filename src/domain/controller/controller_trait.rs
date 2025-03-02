use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;

pub trait Controller<T: StateVector, U: Force> {
    fn compute_control_input(&self, state: &T) -> U;
}
