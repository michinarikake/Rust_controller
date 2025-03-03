use ndarray::{s, Array1};
use std::marker::PhantomData;

use crate::domain::state::state_trait::StateVector;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::cost::cost_trait::Cost;
use crate::domain::force::force_trait::Force;

/// **ModeScheduler 用の拡張状態量**
#[derive(Clone)]
pub struct ModeState<T: StateVector> {
    combined: Array1<f64>,  // 評価関数の値と元の状態量を結合したベクトル
    cost_values: f64,       // 評価関数の値
    original_state: T,      // もとの状態量
}

impl<T: StateVector> ModeState<T> {
    pub fn new(cost_values: f64, original_state: T) -> Self {
        let mut combined = Array1::<f64>::zeros(1 + original_state.get_vector().len());
        combined[0] = cost_values;
        combined.slice_mut(s![1..]).assign(&original_state.get_vector());

        Self {
            combined,
            cost_values,
            original_state,
        }
    }

    pub fn get_cost_values(&self) -> f64 {
        self.cost_values
    }

    pub fn get_original_state(&self) -> &T {
        &self.original_state
    }
}

impl<T: StateVector> StateVector for ModeState<T> {
    fn get_vector(&self) -> &Array1<f64> {
        &self.combined
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        let cost_values = vec[0];
        let original_state = T::form_from_array(vec.slice(s![1..]).to_owned());
        
        Self {
            combined: vec,
            cost_values,
            original_state,
        }
    }
}

pub trait InputDefinedDynamics<T: StateVector, U: Force> {
    fn get_input(&self, state: &T, t: f64) -> U;
}

/// **ModeScheduler 用の拡張ダイナミクス**
pub struct ModeDynamics<T, U, C, D>
where
    T: StateVector,
    U: Force,
    C: Cost<T, U>,
    D: ContinuousDynamics<T, U>,
{
    cost_function: C,
    base_dynamics: D,
    _marker: PhantomData<T>,
    _marker2: PhantomData<U>,
}

impl<T, U, C, D> ModeDynamics<T, U, C, D>
where
    T: StateVector,
    U: Force,
    C: Cost<T, U>,
    D: ContinuousDynamics<T, U>,
{
    pub fn new(cost_function: C, base_dynamics: D) -> Self {
        Self { cost_function, base_dynamics, _marker: PhantomData, _marker2: PhantomData }
    }
}

impl<T, U, C, D> ContinuousDynamics<ModeState<T>, U> for ModeDynamics<T, U, C, D>
where
    T: StateVector,
    U: Force,
    C: Cost<T, U>,
    D: ContinuousDynamics<T, U>,
{
    fn compute_derivative(&self, mode_state: &ModeState<T>, input: &U) -> ModeState<T> {
        let original_state = mode_state.get_original_state();
        let cost_derivative = self.cost_function.stage_cost(original_state, input);
        let state_derivative = self.base_dynamics.compute_derivative(original_state, input);

        ModeState::new(cost_derivative, state_derivative)
    }
}
