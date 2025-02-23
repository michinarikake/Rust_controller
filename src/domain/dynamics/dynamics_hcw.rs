use ndarray::{Array1, Array2, arr2, s};

use crate::domain::force::force_3d::Force3D;
use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::state::relative_position_velocity_state::RelativePositionVelocityState;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;

/// **二体問題の連続ダイナミクス**
#[derive(Debug, Clone)]
pub struct HcwDynamics {
    mu: f64, // 地球の重力定数
    a: f64, // 軌道長半径
    n: f64 // 平均角速度
}

impl HcwDynamics {
    pub fn new(mu: f64, a: f64) -> Self {
        Self { 
            mu: mu,
            a : a,
            n: (mu / a.powf(3.0)).powf(0.5)
         }
    }
}

impl ContinuousDynamics<RelativePositionVelocityState, Force3D> for HcwDynamics {
    fn compute_derivative(&self, state: &RelativePositionVelocityState, input: &Force3D) -> RelativePositionVelocityState {
        let system_matrix = arr2(&[
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [3.0 * self.n.powf(2.0), 0.0, 0.0, 0.0, 2.0 * self.n, 0.0],
            [0.0, 0.0, 0.0, -2.0 * self.n, 0.0, 0.0],
            [0.0, 0.0, -self.n.powf(2.0), 0.0, 0.0, 0.0]]);
        let input_matrix = arr2(&[
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]);
        let vec = (system_matrix * state.clone()).get_vector() + (input_matrix * input.clone()).get_vector();
        RelativePositionVelocityState::form_from_array(vec)
    }
}