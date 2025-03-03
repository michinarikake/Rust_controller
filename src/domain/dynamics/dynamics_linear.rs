use ndarray::Array2;
use crate::domain::state::state_trait::StateVector;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::force::force_trait::Force;
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::differentiable::differentiable_trait::Differentiable2d;

/// **線形ダイナミクス: \(\dot{x} = A x + B u\)**
pub struct LinearDynamics {
    a_matrix: Array2<f64>, // 状態遷移行列 A
    b_matrix: Array2<f64>, // 入力行列 B
}

impl LinearDynamics {
    pub fn new(a_matrix: Array2<f64>, b_matrix: Array2<f64>) -> Self {
        Self { a_matrix, b_matrix }
    }
}

impl ContinuousDynamics<PositionVelocityStateEci, Force3dEci> for LinearDynamics
{
    fn compute_derivative(&self, state: &PositionVelocityStateEci, input: &Force3dEci) -> PositionVelocityStateEci {
        let x_vec = state.get_vector();
        let u_vec = input.get_vector();
        let dx = self.a_matrix.dot(x_vec) + self.b_matrix.dot(u_vec);
        PositionVelocityStateEci::form_from_array(dx)
    }
}

impl<T, U> Differentiable2d<T, U> for LinearDynamics
where
    T: StateVector,
    U: Force,
{
    fn differentiate(&self, _x: &T, _u: &U, _t: f64) -> Array2<f64> {
        self.a_matrix.clone() // 状態方程式の微分 (\(\nabla_x \dot{x} = A\))
    }
}
