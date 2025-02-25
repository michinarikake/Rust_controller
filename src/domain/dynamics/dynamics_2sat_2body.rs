use ndarray::s;

use crate::domain::force::force_trait::Force;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::state::state_converter::StateConverter;
use crate::domain::state::state_trait::StateVector;
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::force::force_6d_eci::Force6dEci;
use crate::settings::constants::CONSTANTS;

/// **二体問題の連続ダイナミクス**
#[derive(Debug, Clone)]
pub struct PairTwoBodyDynamics {}

impl PairTwoBodyDynamics {
    pub fn new() -> Self {
        Self {}
    }
}

impl ContinuousDynamics<PositionVelocityPairStateEci, Force6dEci> for PairTwoBodyDynamics {
    fn compute_derivative(&self, state: &PositionVelocityPairStateEci, input: &Force6dEci) -> PositionVelocityPairStateEci {
        let mu = CONSTANTS.mu;
        let state_vec: Vec<PositionVelocityStateEci> = state.convert();
        let state_chief = &state_vec[0];
        let state_deputy = &state_vec[1];

        let r_vec_chief = state_chief.position();
        let v_vec_chief = state_chief.velocity();
        let r_norm_chief = state_chief.position_norm();
        let a_vec_chief = -mu / (r_norm_chief.powf(3.0)) * r_vec_chief + input.get_vector().slice(s![0..3]).to_owned();

        let r_vec_deputy = state_deputy.position();
        let v_vec_deputy = state_deputy.velocity();
        let r_norm_deputy =state_deputy.position_norm();
        let a_vec_deputy = -mu / (r_norm_deputy.powf(3.0)) * r_vec_deputy + input.get_vector().slice(s![3..6]).to_owned();

        PositionVelocityPairStateEci::form_from_array(ndarray::concatenate![ndarray::Axis(0), v_vec_chief, a_vec_chief, v_vec_deputy, a_vec_deputy])
    }
}