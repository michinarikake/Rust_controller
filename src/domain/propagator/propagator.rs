use crate::domain::state::state_trait::StateVector;
use std::ops::{Add, Sub, Mul, Div};

pub trait Propagator<T: StateVector> {
    fn propagate(&self, state: &T, dynamics: &dyn Fn(&T) -> T, dt: f64) -> T;
}

pub struct EulerPropagator;

impl<T> Propagator<T> for EulerPropagator 
where
    T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T>,
{
    fn propagate(&self, state: &T, dynamics: &dyn Fn(&T) -> T, dt: f64) -> T {
        let derivative = dynamics(state);
        state.clone() + derivative * dt
    }
}

pub struct RungeKutta4Propagator;

impl<T> Propagator<T> for RungeKutta4Propagator 
where
    T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T>,
{
    fn propagate(&self, state: &T, dynamics: &dyn Fn(&T) -> T, dt: f64) -> T {
        let k1 = dynamics(state);
        let k2 = dynamics(&(state.clone() + k1.clone() * (0.5 * dt)));
        let k3 = dynamics(&(state.clone() + k2.clone() * (0.5 * dt)));
        let k4 = dynamics(&(state.clone() + k3.clone() * (dt)));

        state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4 * dt) / 6.0
    }
}
