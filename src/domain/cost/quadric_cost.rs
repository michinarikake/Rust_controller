use ndarray::{Array1, Array2};
use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;
use crate::domain::cost::cost_trait::Cost;
use crate::domain::differentiable::differentiable_trait::Differentiable1d;

/// **LQR の 2 次形式のコスト関数**
pub struct QuadraticCost {
    q_matrix: Array2<f64>,  // 状態の重み行列 Q
    r_matrix: Array2<f64>,  // 制御入力の重み行列 R
    qf_matrix: Array2<f64>, // 終端コストの重み行列 Q_f
    t_last: f64,            // 終端時刻
}

impl QuadraticCost {
    pub fn new(q_matrix: Array2<f64>, r_matrix: Array2<f64>, qf_matrix: Array2<f64>, t_last: f64) -> Self {
        Self {
            q_matrix,
            r_matrix,
            qf_matrix,
            t_last,
        }
    }
}

impl<T, U> Cost<T, U> for QuadraticCost
where
    T: StateVector,
    U: Force,
{
    fn stage_cost(&self, x: &T, v: &U) -> f64 {
        let x_vec = x.get_vector();
        let v_vec = v.get_vector();
        let x_cost = x_vec.dot(&(self.q_matrix.dot(x_vec)));
        let v_cost = v_vec.dot(&(self.r_matrix.dot(v_vec)));
        x_cost + v_cost
    }

    fn terminal_cost(&self, x: &T) -> f64 {
        let x_vec = x.get_vector();
        x_vec.dot(&(self.qf_matrix.dot(x_vec)))
    }
}

impl<T, U> Differentiable1d<T, U> for QuadraticCost
where
    T: StateVector,
    U: Force,
{
    #[allow(unused_variables)]
    fn differentiate(&self, x: &T, v: &U, t: f64) -> Array1<f64> {
        let x_vec = x.get_vector();
        if t >= self.t_last {
            // 終端コストの微分
            self.qf_matrix.dot(x_vec)
        } else {
            // ステージコストの微分
            let x_grad = self.q_matrix.dot(x_vec);
            x_grad
        }
    }
}

#[cfg(test)]
use ndarray::arr1;
#[cfg(test)]
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
#[cfg(test)]
use crate::domain::force::force_3d_eci::Force3dEci;

#[test]
fn test_quadratic_cost() {
    // 状態と制御の次元数
    let state_dim = 6;  // 修正: 状態次元を 6 に統一
    let control_dim = 3; // 修正: 制御次元を 3 に統一

    // Q, R, Q_f の行列を定義
    let q_matrix = Array2::<f64>::eye(state_dim);
    let r_matrix = Array2::<f64>::eye(control_dim);
    let qf_matrix = Array2::<f64>::eye(state_dim);

    // 終端時刻
    let t_last = 10.0;

    // コスト関数のインスタンスを作成
    let cost_function = QuadraticCost::new(q_matrix, r_matrix, qf_matrix, t_last);

    // 状態と制御入力を仮定
    let x = PositionVelocityPairStateEci::form_from_array(arr1(&[1.0, 2.0, 3.0, 0.0, 0.0, 0.0])); // 修正: 状態の次元を6に
    let v = Force3dEci::form_from_array(arr1(&[0.5, 1.5, 0.0])); // 修正: 制御の次元を3に

    // t = 5.0 での勾配
    let grad_before_t_last = cost_function.differentiate(&x, &v, 5.0);
    println!("Gradient at t=5.0: {:?}", grad_before_t_last);

    // t = 10.0 (終端) での勾配
    let grad_at_t_last = cost_function.differentiate(&x, &v, 10.0);
    println!("Gradient at t=10.0: {:?}", grad_at_t_last);
}
