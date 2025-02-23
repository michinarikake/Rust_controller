use ndarray::{Array1, Array2, arr1, s};
use std::ops::{Add, Sub, Mul, Div};

use super::state_trait::StateVector;
use super::relative_position_velocity_state::RelativePositionVelocityState;
use crate::repositry::loggable::Loggable;

// 位置・速度・共分散の状態量
#[derive(Debug, Clone)]
pub struct RelativePositionVelocityCovarianceState {
    state: Array1<f64>, // [l, mu_x(6), est_x(6), p(21)]
}

impl RelativePositionVelocityCovarianceState {
    pub fn form_from_list(l: f64, mu_x: [f64; 6], est_x: [f64; 6], p: Array2<f64>) -> Self {
        assert!(p.shape() == [6, 6], "p must be a 6x6 matrix");

        let mut p_upper = Vec::with_capacity(21);
        for i in 0..6 {
            for j in i..6 {
                p_upper.push(p[[i, j]]);
            }
        }

        let mut state_vec = vec![l];
        state_vec.extend(&mu_x);
        state_vec.extend(&est_x);
        state_vec.extend(p_upper);

        Self {
            state: arr1(&state_vec),
        }
    }

    pub fn from_from_states(l: f64, mu_x: &RelativePositionVelocityState, est_x: &RelativePositionVelocityState, p: Array2<f64>) -> Self {
        assert!(p.shape() == [6, 6], "p must be a 6x6 matrix");

        let mut p_upper = Vec::with_capacity(21);
        for i in 0..6 {
            for j in i..6 {
                p_upper.push(p[[i, j]]);
            }
        }

        let mut state_vec = vec![l];
        state_vec.extend(mu_x.get_vector().iter());
        state_vec.extend(est_x.get_vector().iter());
        state_vec.extend(p_upper);

        Self {
            state: arr1(&state_vec),
        }
    }

    pub fn get_l(&self) -> f64 {
        self.state[0]
    }

    pub fn get_mu_x(&self) -> RelativePositionVelocityState {
        let position: [f64; 3] = self.state.slice(s![1..4]).to_owned().to_vec().try_into().unwrap();
        let velocity: [f64; 3] = self.state.slice(s![4..7]).to_owned().to_vec().try_into().unwrap();
        RelativePositionVelocityState::form_from_list(position, velocity)
    }

    pub fn get_est_x(&self) -> RelativePositionVelocityState {
        let position: [f64; 3] = self.state.slice(s![7..10]).to_owned().to_vec().try_into().unwrap();
        let velocity: [f64; 3] = self.state.slice(s![10..13]).to_owned().to_vec().try_into().unwrap();
        RelativePositionVelocityState::form_from_list(position, velocity)
    }

    pub fn get_covariance_matrix(&self) -> Array2<f64> {
        let mut p_full = Array2::<f64>::zeros((6, 6));
        let p_data = &self.state.as_slice().unwrap()[13..];

        let mut index = 0;
        for i in 0..6 {
            for j in i..6 {
                p_full[[i, j]] = p_data[index];
                p_full[[j, i]] = p_data[index]; // 対称性を反映
                index += 1;
            }
        }
        p_full
    }
}


impl StateVector for RelativePositionVelocityCovarianceState {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { state: vec }
    }

}

impl Loggable for RelativePositionVelocityCovarianceState{
    fn header(&self) -> String {
        let mu_headers: Vec<String> = (0..6).map(|i| format!("mu_x_{}", i)).collect();
        let est_headers: Vec<String> = (0..6).map(|i| format!("est_x_{}", i)).collect();
        let p_headers: Vec<String> = (0..21).map(|i| format!("p_{}", i)).collect();

        format!("l,{}, {}, {}", mu_headers.join(","), est_headers.join(","), p_headers.join(","))
    }

    /// **ログの出力**
    fn output_log(&self) -> String {
        let l = self.get_l();
        let mu_x = self.get_mu_x().get_vector().iter().map(|v| v.to_string()).collect::<Vec<_>>().join(",");
        let est_x = self.get_est_x().get_vector().iter().map(|v| v.to_string()).collect::<Vec<_>>().join(",");
        let covariance = self.get_covariance_matrix().iter().map(|v| v.to_string()).collect::<Vec<_>>().join(",");

        format!("{},{},{},{}", l, mu_x, est_x, covariance)
    }
}

/// **演算子のオーバーロード**
impl Add for RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn add(self, rhs: RelativePositionVelocityCovarianceState) -> RelativePositionVelocityCovarianceState {
        self.add_vec(&rhs)
    }
}

impl Add for &RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn add(self, rhs: &RelativePositionVelocityCovarianceState) -> RelativePositionVelocityCovarianceState {
        self.add_vec(&rhs)
    }
}

impl Sub for RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn sub(self, rhs: RelativePositionVelocityCovarianceState) -> RelativePositionVelocityCovarianceState {
        self.sub_vec(&rhs)
    }
}

impl Sub for &RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn sub(self, rhs: &RelativePositionVelocityCovarianceState) -> RelativePositionVelocityCovarianceState {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn mul(self, scalar: f64) -> RelativePositionVelocityCovarianceState {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn mul(self, scalar: f64) -> RelativePositionVelocityCovarianceState {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn div(self, scalar: f64) -> RelativePositionVelocityCovarianceState {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &RelativePositionVelocityCovarianceState {
    type Output = RelativePositionVelocityCovarianceState;
    fn div(self, scalar: f64) -> RelativePositionVelocityCovarianceState {
        self.div_scalar(scalar)
    }
}

impl Mul<RelativePositionVelocityCovarianceState> for Array2<f64> {
    type Output = RelativePositionVelocityCovarianceState;
    fn mul(self, rhs: RelativePositionVelocityCovarianceState) -> RelativePositionVelocityCovarianceState {
        let result = self.dot(rhs.get_vector());
        RelativePositionVelocityCovarianceState::form_from_array(result)
    }
}


/// **`RelativePositionVelocityCovarianceState` の共分散行列のテスト**
use ndarray::{arr2};
#[test]
fn test_position_velocity_covariance_state() {
    let p_full = arr2(&[
        [0.1, 0.01, 0.02, 0.03, 0.04, 0.05],
        [0.01, 0.2, 0.06, 0.07, 0.08, 0.09],
        [0.02, 0.06, 0.3, 0.10, 0.11, 0.12],
        [0.03, 0.07, 0.10, 0.4, 0.13, 0.14],
        [0.04, 0.08, 0.11, 0.13, 0.5, 0.15],
        [0.05, 0.09, 0.12, 0.14, 0.15, 0.6],
    ]);

    let mu_x = RelativePositionVelocityState::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);
    let est_x = RelativePositionVelocityState::form_from_list([6999.0, 0.0, 0.1], [0.0, 7.49, 0.1]);

    let pvc_state = RelativePositionVelocityCovarianceState::from_from_states(1.0, &mu_x, &est_x, p_full.clone());

    assert_eq!(pvc_state.get_l(), 1.0);
    assert_eq!(pvc_state.get_mu_x().get_vector(), mu_x.get_vector());
    assert_eq!(pvc_state.get_est_x().get_vector(), est_x.get_vector());
    assert_eq!(pvc_state.get_covariance_matrix(), p_full);
}
