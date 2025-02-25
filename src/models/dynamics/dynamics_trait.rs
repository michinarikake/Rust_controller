use crate::models::state::state_trait::StateVector;
use crate::models::force::force_trait::Force;

/// **連続ダイナミクスのトレイト**
pub trait ContinuousDynamics<T: StateVector, U: Force> {
    fn compute_derivative(&self, state: &T, input: &U) -> T;
}

/// **離散ダイナミクスのトレイト**
pub trait DiscreteDynamics<T: StateVector, U: Force> {
    fn step(&self, state: &T, dt: f64) -> T;
}