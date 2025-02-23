use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::state::position_velocity_pair_state_ecef::PositionVelocityPairStateEcef;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::force::force_3d_ecef::Force3dEcef;

/// **二体問題の連続ダイナミクス**
#[derive(Debug, Clone)]
pub struct PairTwoBodyDynamics {
    mu: f64, // 地球の重力定数
}

impl PairTwoBodyDynamics {
    pub fn new(mu: f64) -> Self {
        Self { mu }
    }
}

impl ContinuousDynamics<PositionVelocityPairStateEcef, Force3dEcef> for PairTwoBodyDynamics {
    fn compute_derivative(&self, state: &PositionVelocityPairStateEcef, input: &Force3dEcef) -> PositionVelocityPairStateEcef {
        let r_vec_chief = state.chief().position();
        let v_vec_chief = state.chief().velocity();
        let r_norm_chief = state.chief().position_norm();
        let a_vec_chief = -self.mu / (r_norm_chief.powi(3)) * r_vec_chief + input.get_vector();

        let r_vec_deputy = state.deputy().position();
        let v_vec_deputy = state.deputy().velocity();
        let r_norm_deputy = state.deputy().position_norm();
        let a_vec_deputy = -self.mu / (r_norm_deputy.powi(3)) * r_vec_deputy + input.get_vector();

        PositionVelocityPairStateEcef::form_from_array(ndarray::concatenate![ndarray::Axis(0), v_vec_chief, a_vec_chief, v_vec_deputy, a_vec_deputy])
    }
}