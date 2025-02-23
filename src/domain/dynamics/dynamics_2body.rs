use ndarray::{s};

use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::state::position_velocity_state_ecef::PositionVelocityStateEcef;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::force::force_3d_ecef::Force3dEcef;

/// **二体問題の連続ダイナミクス**
#[derive(Debug, Clone)]
pub struct TwoBodyDynamics {
    mu: f64, // 地球の重力定数
}

impl TwoBodyDynamics {
    pub fn new(mu: f64) -> Self {
        Self { mu }
    }
}

impl ContinuousDynamics<PositionVelocityStateEcef, Force3dEcef> for TwoBodyDynamics {
    fn compute_derivative(&self, state: &PositionVelocityStateEcef, input: &Force3dEcef) -> PositionVelocityStateEcef {
        let r_vec = state.position();
        let v_vec = state.velocity();
        let r_norm = state.position_norm();
        let a_vec = -self.mu / (r_norm.powi(3)) * r_vec + input.get_vector();

        PositionVelocityStateEcef::form_from_array(ndarray::concatenate![ndarray::Axis(0), v_vec, a_vec])
    }
}