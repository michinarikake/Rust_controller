use ndarray::{Array1, Array2, arr1, s};
use std::ops::{Add, Sub, Mul, Div};

use super::state_trait::StateVector;
use super::position_velocity_state::PositionVelocityState;


// 位置・速度の状態量
#[derive(Debug, Clone)]
pub struct RelativePositionVelocityState {
    state: Array1<f64>, // [px, py, pz, vx, vy, vz]
}

impl RelativePositionVelocityState {
    pub fn form_from_list(position: [f64; 3], velocity: [f64; 3]) -> Self {
        let state = arr1(&[position[0], position[1], position[2], velocity[0], velocity[1], velocity[2]]);
        Self { state }
    }

    pub fn form_from_states(state_base: &PositionVelocityState, state_target: &PositionVelocityState) -> Self {
        let rel_position = state_target.position() - state_base.position();
        let rel_velocity = state_target.velocity() - state_base.velocity();

        Self::form_from_array(ndarray::concatenate![ndarray::Axis(0), rel_position, rel_velocity])
    }

    pub fn position(&self) -> Array1<f64> {
        self.state.slice(s![0..3]).to_owned()
    }

    pub fn velocity(&self) -> Array1<f64> {
        self.state.slice(s![3..6]).to_owned()
    }

    pub fn position_norm(&self) -> f64 {
        (self.position().dot(&self.position())).sqrt()
    }

    pub fn velocity_norm(&self) -> f64 {
        (self.velocity().dot(&self.velocity())).sqrt()
    }
}

impl StateVector for RelativePositionVelocityState {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { state: vec }
    }
}

/// **演算子のオーバーロード**
impl Add for RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn add(self, rhs: RelativePositionVelocityState) -> RelativePositionVelocityState {
        self.add_vec(&rhs)
    }
}

impl Add for &RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn add(self, rhs: &RelativePositionVelocityState) -> RelativePositionVelocityState {
        self.add_vec(&rhs)
    }
}

impl Sub for RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn sub(self, rhs: RelativePositionVelocityState) -> RelativePositionVelocityState {
        self.sub_vec(&rhs)
    }
}

impl Sub for &RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn sub(self, rhs: &RelativePositionVelocityState) -> RelativePositionVelocityState {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn mul(self, scalar: f64) -> RelativePositionVelocityState {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn mul(self, scalar: f64) -> RelativePositionVelocityState {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn div(self, scalar: f64) -> RelativePositionVelocityState {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &RelativePositionVelocityState {
    type Output = RelativePositionVelocityState;
    fn div(self, scalar: f64) -> RelativePositionVelocityState {
        self.div_scalar(scalar)
    }
}

impl Mul<RelativePositionVelocityState> for Array2<f64> {
    type Output = RelativePositionVelocityState;
    fn mul(self, rhs: RelativePositionVelocityState) -> RelativePositionVelocityState {
        let result = self.dot(rhs.get_vector());
        RelativePositionVelocityState::form_from_array(result)
    }
}
use ndarray::{arr2};
/// **`RelativePositionVelocityState` の基本演算テスト**
#[test]
fn test_position_velocity_state_operations() {
    let pv_state1 = RelativePositionVelocityState::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);
    let pv_state2 = RelativePositionVelocityState::form_from_list([1000.0, 0.0, 0.0], [0.0, -1.5, 0.0]);

    // 加算
    let sum = pv_state1.clone() + pv_state2.clone();
    assert_eq!(sum.get_vector(), &arr1(&[8000.0, 0.0, 0.0, 0.0, 6.0, 0.0]));

    // 減算
    let diff = pv_state1.clone() - pv_state2.clone();
    assert_eq!(diff.get_vector(), &arr1(&[6000.0, 0.0, 0.0, 0.0, 9.0, 0.0]));

    // スカラー乗算
    let scaled = pv_state1.clone() * 2.0;
    assert_eq!(scaled.get_vector(), &arr1(&[14000.0, 0.0, 0.0, 0.0, 15.0, 0.0]));

    // スカラー除算
    let divided = pv_state1.clone() / 2.0;
    assert_eq!(divided.get_vector(), &arr1(&[3500.0, 0.0, 0.0, 0.0, 3.75, 0.0]));
}


/// **行列 x `RelativePositionVelocityState` の変換テスト**
#[test]
fn test_position_velocity_state_matrix_multiplication() {
    let pv_state = RelativePositionVelocityState::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);

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

#[test]
fn test_form_from_states() {
    let base = PositionVelocityState::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.0, 0.0]);
    let target = PositionVelocityState::form_from_list([7100.0, 100.0, 0.0], [0.1, 8.0, 0.0]);

    let relative = RelativePositionVelocityState::form_from_states(&base, &target);

    assert_eq!(relative.position(), arr1(&[100.0, 100.0, 0.0]));
    assert_eq!(relative.velocity(), arr1(&[0.1, 1.0, 0.0]));
}
