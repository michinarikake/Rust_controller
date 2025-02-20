use ndarray::{Array1, Array2, arr1, arr2, s};
use std::ops::{Add, Sub, Mul, Div};

use super::state_trait::StateVector;
use super::position_velocity_state::PositionVelocityState;

// 位置・速度・共分散の状態量
#[derive(Debug, Clone)]
pub struct PositionVelocityCovarianceState {
    state: Array1<f64>, // [l, mu_x(6), est_x(6), p(21)]
}

impl PositionVelocityCovarianceState {
    pub fn new(l: f64, mu_x: [f64; 6], est_x: [f64; 6], p: Array2<f64>) -> Self {
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

    pub fn from_states(l: f64, mu_x: &PositionVelocityState, est_x: &PositionVelocityState, p: Array2<f64>) -> Self {
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

    pub fn get_mu_x(&self) -> PositionVelocityState {
        let position: [f64; 3] = self.state.slice(s![1..4]).to_owned().to_vec().try_into().unwrap();
        let velocity: [f64; 3] = self.state.slice(s![4..7]).to_owned().to_vec().try_into().unwrap();
        PositionVelocityState::new(position, velocity)
    }

    pub fn get_est_x(&self) -> PositionVelocityState {
        let position: [f64; 3] = self.state.slice(s![7..10]).to_owned().to_vec().try_into().unwrap();
        let velocity: [f64; 3] = self.state.slice(s![10..13]).to_owned().to_vec().try_into().unwrap();
        PositionVelocityState::new(position, velocity)
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


impl StateVector for PositionVelocityCovarianceState {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn from_array(vec: Array1<f64>) -> Self {
        Self { state: vec }
    }
}

/// **演算子のオーバーロード**
impl Add for PositionVelocityCovarianceState {
    type Output = PositionVelocityCovarianceState;
    fn add(self, rhs: PositionVelocityCovarianceState) -> PositionVelocityCovarianceState {
        self.add_vec(rhs)
    }
}

impl Sub for PositionVelocityCovarianceState {
    type Output = PositionVelocityCovarianceState;
    fn sub(self, rhs: PositionVelocityCovarianceState) -> PositionVelocityCovarianceState {
        self.sub_vec(rhs)
    }
}

impl Mul<f64> for PositionVelocityCovarianceState {
    type Output = PositionVelocityCovarianceState;
    fn mul(self, scalar: f64) -> PositionVelocityCovarianceState {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for PositionVelocityCovarianceState {
    type Output = PositionVelocityCovarianceState;
    fn div(self, scalar: f64) -> PositionVelocityCovarianceState {
        self.div_scalar(scalar)
    }
}

impl Mul<PositionVelocityCovarianceState> for Array2<f64> {
    type Output = PositionVelocityCovarianceState;
    fn mul(self, rhs: PositionVelocityCovarianceState) -> PositionVelocityCovarianceState {
        let result = self.dot(rhs.get_vector());
        PositionVelocityCovarianceState::from_array(result)
    }
}

/// **行列 x `PositionVelocityState` の変換テスト**
#[test]
fn test_position_velocity_state_matrix_multiplication() {
    let pv_state = PositionVelocityState::new([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);

    let transform_matrix = arr2(&[
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]);

    let transformed = transform_matrix * pv_state.clone();
    assert_eq!(transformed.get_vector(), pv_state.get_vector());
}

/// **`PositionVelocityCovarianceState` の共分散行列のテスト**
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

    let mu_x = PositionVelocityState::new([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);
    let est_x = PositionVelocityState::new([6999.0, 0.0, 0.1], [0.0, 7.49, 0.1]);

    let pvc_state = PositionVelocityCovarianceState::from_states(1.0, &mu_x, &est_x, p_full.clone());

    assert_eq!(pvc_state.get_l(), 1.0);
    assert_eq!(pvc_state.get_mu_x().get_vector(), mu_x.get_vector());
    assert_eq!(pvc_state.get_est_x().get_vector(), est_x.get_vector());
    assert_eq!(pvc_state.get_covariance_matrix(), p_full);
}
