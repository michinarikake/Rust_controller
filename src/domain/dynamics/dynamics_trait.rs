use crate::domain::state::state_trait::StateVector;

/// **連続ダイナミクスのトレイト**
pub trait ContinuousDynamics<T: StateVector> {
    fn compute_derivative(&self, state: &T) -> T;
}

/// **離散ダイナミクスのトレイト**
pub trait DiscreteDynamics<T: StateVector> {
    fn step(&self, state: &T, dt: f64) -> T;
}