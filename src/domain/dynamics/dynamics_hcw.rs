use ndarray::arr2;

use crate::domain::force::force_3d_lvlh::Force3dLvlh;
use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::settings::constants::CONSTANTS;

/// **二体問題の連続ダイナミクス**
#[derive(Debug, Clone)]
pub struct HcwDynamics {
    n: f64 // 平均角速度
}

impl HcwDynamics {
    pub fn new(a: f64) -> Self {
        Self { 
            n: (CONSTANTS.mu / a.powf(3.0)).powf(0.5)
         }
    }
}

impl ContinuousDynamics<PositionVelocityStateLvlh, Force3dLvlh> for HcwDynamics {
    fn compute_derivative(&self, state: &PositionVelocityStateLvlh, input: &Force3dLvlh) -> PositionVelocityStateLvlh {
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
        PositionVelocityStateLvlh::form_from_array(vec)
    }
}