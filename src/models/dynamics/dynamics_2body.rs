use crate::models::force::force_trait::Force;
use crate::models::state::state_trait::StateVector;
use crate::models::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::models::dynamics::dynamics_trait::ContinuousDynamics;
use crate::models::force::force_3d_eci::Force3dEci;
use crate::settings::constants::CONSTANTS;

/// **二体問題の連続ダイナミクス**
#[derive(Debug, Clone)]
pub struct TwoBodyDynamics {}

impl TwoBodyDynamics {
    pub fn new() -> Self {
        Self {}
    }
}

impl ContinuousDynamics<PositionVelocityStateEci, Force3dEci> for TwoBodyDynamics {
    fn compute_derivative(&self, state: &PositionVelocityStateEci, input: &Force3dEci) -> PositionVelocityStateEci {
        let mu = CONSTANTS.mu;
        let r_vec = state.position();
        let v_vec = state.velocity();
        let r_norm = state.position_norm();
        let a_vec = -mu / (r_norm.powi(3)) * r_vec + input.get_vector();

        PositionVelocityStateEci::form_from_array(ndarray::concatenate![ndarray::Axis(0), v_vec, a_vec])
    }
}