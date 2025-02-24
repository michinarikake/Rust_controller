use ndarray::{Array1, Array2, arr1, s};
use std::ops::{Add, Sub, Mul, Div};

use super::state_trait::StateVector;
use super::position_velocity_state_eci::PositionVelocityStateEci;
use crate::repositry::loggable_trait::Loggable;


// 位置・速度の状態量
#[derive(Debug, Clone)]
pub struct PositionVelocityStateLvlh {
    state: Array1<f64>, // [px, py, pz, vx, vy, vz]
}

impl PositionVelocityStateLvlh {
    pub fn form_from_list(position: [f64; 3], velocity: [f64; 3]) -> Self {
        let state = arr1(&[position[0], position[1], position[2], velocity[0], velocity[1], velocity[2]]);
        Self { state }
    }

    pub fn form_from_states(state_base: &PositionVelocityStateEci, state_target: &PositionVelocityStateEci) -> Self {
        Self::form_from_array((state_target - state_base).get_vector().clone())
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

impl StateVector for PositionVelocityStateLvlh {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { state: vec }
    }
}

impl Loggable for PositionVelocityStateLvlh{
    fn output_log(&self) -> String {
        let state_str : Vec<String> = self.get_vector().iter().map(|v| v.to_string()).collect();
        state_str.join(",")
    }

    fn header(&self) -> String {
        "p0,p1,p2,v0,v1,v2".to_string()
    }
}

/// **演算子のオーバーロード**
impl Add for PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn add(self, rhs: PositionVelocityStateLvlh) -> PositionVelocityStateLvlh {
        self.add_vec(&rhs)
    }
}

impl Add for &PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn add(self, rhs: &PositionVelocityStateLvlh) -> PositionVelocityStateLvlh {
        self.add_vec(&rhs)
    }
}

impl Sub for PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn sub(self, rhs: PositionVelocityStateLvlh) -> PositionVelocityStateLvlh {
        self.sub_vec(&rhs)
    }
}

impl Sub for &PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn sub(self, rhs: &PositionVelocityStateLvlh) -> PositionVelocityStateLvlh {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn mul(self, scalar: f64) -> PositionVelocityStateLvlh {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn mul(self, scalar: f64) -> PositionVelocityStateLvlh {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn div(self, scalar: f64) -> PositionVelocityStateLvlh {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &PositionVelocityStateLvlh {
    type Output = PositionVelocityStateLvlh;
    fn div(self, scalar: f64) -> PositionVelocityStateLvlh {
        self.div_scalar(scalar)
    }
}

impl Mul<PositionVelocityStateLvlh> for Array2<f64> {
    type Output = PositionVelocityStateLvlh;
    fn mul(self, rhs: PositionVelocityStateLvlh) -> PositionVelocityStateLvlh {
        let result = self.dot(rhs.get_vector());
        PositionVelocityStateLvlh::form_from_array(result)
    }
}

#[cfg(test)]
use ndarray::arr2;
/// **`PositionVelocityStateLvlh` の基本演算テスト**
#[test]
fn test_position_velocity_state_operations() {
    let pv_state1 = PositionVelocityStateLvlh::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);
    let pv_state2 = PositionVelocityStateLvlh::form_from_list([1000.0, 0.0, 0.0], [0.0, -1.5, 0.0]);

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


/// **行列 x `PositionVelocityStateLvlh` の変換テスト**
#[test]
fn test_position_velocity_state_matrix_multiplication() {
    let pv_state = PositionVelocityStateLvlh::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);

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
    let base = PositionVelocityStateEci::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.0, 0.0]);
    let target = PositionVelocityStateEci::form_from_list([7100.0, 100.0, 0.0], [0.1, 8.0, 0.0]);

    let relative = PositionVelocityStateLvlh::form_from_states(&base, &target);

    assert_eq!(relative.position(), arr1(&[100.0, 100.0, 0.0]));
    assert_eq!(relative.velocity(), arr1(&[0.1, 1.0, 0.0]));
}
