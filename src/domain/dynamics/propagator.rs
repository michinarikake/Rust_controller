use crate::domain::state::state_trait::StateVector;
use crate::domain::dynamics::dynamics_trait::{ContinuousDynamics, DiscreteDynamics};
use std::ops::{Add, Sub, Mul, Div};

/// **伝搬トレイト**
pub trait Propagator<T: StateVector> {
    fn propagate_continuous(&self, state: &T, dynamics: &dyn ContinuousDynamics<T>, dt: f64) -> T;
    fn propagate_discrete(&self, state: &T, dynamics: &dyn DiscreteDynamics<T>, dt: f64) -> T {
        dynamics.step(state, dt)
    }
}

/// **オイラー法**
pub struct EulerPropagator;

impl<T> Propagator<T> for EulerPropagator
where
    T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T> + Clone,
{
    fn propagate_continuous(&self, state: &T, dynamics: &dyn ContinuousDynamics<T>, dt: f64) -> T {
        let derivative = dynamics.compute_derivative(state);
        state.clone() + derivative * dt
    }
}

/// **ルンゲクッタ4次**
pub struct RungeKutta4Propagator;

impl<T> Propagator<T> for RungeKutta4Propagator
where
    T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T> + Clone,
{
    fn propagate_continuous(&self, state: &T, dynamics: &dyn ContinuousDynamics<T>, dt: f64) -> T {
        let k1 = dynamics.compute_derivative(state);
        let k2 = dynamics.compute_derivative(&(state.clone() + k1.clone() * (0.5 * dt)));
        let k3 = dynamics.compute_derivative(&(state.clone() + k2.clone() * (0.5 * dt)));
        let k4 = dynamics.compute_derivative(&(state.clone() + k3.clone() * dt));

        state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0)
    }
}
