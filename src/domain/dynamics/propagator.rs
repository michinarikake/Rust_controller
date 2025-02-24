use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;
use crate::domain::dynamics::dynamics_trait::{ContinuousDynamics, DiscreteDynamics};
use std::ops::{Add, Sub, Mul, Div};

/// **伝搬トレイト**
pub trait Propagator<T: StateVector, U: Force> {
    fn propagate_continuous(&self, state: &T, input: &U, dynamics: &dyn ContinuousDynamics<T, U>, dt: f64) -> T;
    fn propagate_discrete(&self, state: &T, dynamics: &dyn DiscreteDynamics<T, U>, dt: f64) -> T {
        dynamics.step(state, dt)
    }
    fn new(_: std::marker::PhantomData<(T, U)>) -> Self;
}

/// **オイラー法**
#[derive(Debug, Clone)]
pub struct EulerPropagator;

impl<T, U> Propagator<T, U> for EulerPropagator
where
    T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T> + Clone,
    U: Force + Add<Output = U> + Sub<Output = U> + Mul<f64, Output = U> + Div<f64, Output = U> + Clone,
{
    fn propagate_continuous(&self, state: &T, input: &U, dynamics: &dyn ContinuousDynamics<T, U>, dt: f64) -> T {
        let derivative = dynamics.compute_derivative(state, input);
        state.clone() + derivative * dt
    }

    fn new(_: std::marker::PhantomData<(T, U)>) -> Self {
        Self
    }
}

/// **ルンゲクッタ4次**
#[derive(Debug, Clone)]
pub struct RungeKutta4Propagator;

impl<T, U> Propagator<T, U> for RungeKutta4Propagator
where
    T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T> + Clone,
    U: Force + Add<Output = U> + Sub<Output = U> + Mul<f64, Output = U> + Div<f64, Output = U> + Clone,
{
    fn propagate_continuous(&self, state: &T, input: &U, dynamics: &dyn ContinuousDynamics<T, U>, dt: f64) -> T {
        let k1 = dynamics.compute_derivative(state, input);
        let k2 = dynamics.compute_derivative(&(state.clone() + k1.clone() * (0.5 * dt)), input);
        let k3 = dynamics.compute_derivative(&(state.clone() + k2.clone() * (0.5 * dt)), input);
        let k4 = dynamics.compute_derivative(&(state.clone() + k3.clone() * dt), input);

        state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0)
    }

    fn new(_: std::marker::PhantomData<(T, U)>) -> Self {
        Self
    }
}
