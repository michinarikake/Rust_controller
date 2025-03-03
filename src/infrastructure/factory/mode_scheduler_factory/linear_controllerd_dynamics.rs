use crate::domain::state::{position_velocity_state_eci::PositionVelocityStateEci, state_trait::StateVector};
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::force::force_trait::Force;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::differentiable::differentiable_trait::Differentiable2d;
use crate::domain::controller::mode_controller::wrapper::InputDefinedDynamics;
use ndarray::Array2;

/// **制御入力が固定された線形ダイナミクス**
pub struct LinearControlledDynamics {
    a_matrix: Array2<f64>,
    b_matrix: Array2<f64>,
    control_input: Force3dEci,  // 固定された制御入力
}

impl LinearControlledDynamics {
    pub fn new(control_input: Force3dEci) -> Self {
        let a_matrix = Array2::<f64>::eye(6);  // 仮のA行列（適切に変更）
        let b_matrix = Array2::<f64>::eye(6);  // 仮のB行列（適切に変更）
        Self { a_matrix, b_matrix, control_input }
    }
}

impl ContinuousDynamics<PositionVelocityStateEci, Force3dEci> for LinearControlledDynamics {
    fn compute_derivative(&self, state: &PositionVelocityStateEci, _: &Force3dEci) -> PositionVelocityStateEci {
        let dx = (self.a_matrix.clone() * state).get_vector() + (self.b_matrix.clone() * self.control_input.clone()).get_vector();
        PositionVelocityStateEci::form_from_array(dx)
    }
}

impl Differentiable2d<PositionVelocityStateEci, Force3dEci> for LinearControlledDynamics {
    fn differentiate(&self, _x: &PositionVelocityStateEci, _v: &Force3dEci, _t: f64) -> Array2<f64> {
        self.a_matrix.clone()
    }
}

impl InputDefinedDynamics<PositionVelocityStateEci, Force3dEci> for LinearControlledDynamics {
    fn get_input(&self, state: &PositionVelocityStateEci, t: f64) -> Force3dEci {
        self.control_input.clone()
    }
}
