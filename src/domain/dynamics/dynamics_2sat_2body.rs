use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::settings::constants::CONSTANTS;

/// **二体問題の連続ダイナミクス**
#[derive(Debug, Clone)]
pub struct PairTwoBodyDynamics {}

impl PairTwoBodyDynamics {
    pub fn new() -> Self {
        Self {}
    }
}

impl ContinuousDynamics<PositionVelocityPairStateEci, Force3dEci> for PairTwoBodyDynamics {
    fn compute_derivative(&self, state: &PositionVelocityPairStateEci, input: &Force3dEci) -> PositionVelocityPairStateEci {
        let mu = CONSTANTS.mu;
        let r_vec_chief = state.chief().position();
        let v_vec_chief = state.chief().velocity();
        let r_norm_chief = state.chief().position_norm();
        let a_vec_chief = -mu / (r_norm_chief.powf(3.0)) * r_vec_chief;// ここ力入れてもいいかも。仮想の基準にするかどうか？

        let r_vec_deputy = state.deputy().position();
        let v_vec_deputy = state.deputy().velocity();
        let r_norm_deputy = state.deputy().position_norm();
        let a_vec_deputy = -mu / (r_norm_deputy.powf(3.0)) * r_vec_deputy + input.get_vector();

        PositionVelocityPairStateEci::form_from_array(ndarray::concatenate![ndarray::Axis(0), v_vec_chief, a_vec_chief, v_vec_deputy, a_vec_deputy])
    }
}