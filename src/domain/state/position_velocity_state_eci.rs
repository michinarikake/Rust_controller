use ndarray::{arr1, s, Array1, Array2};
use std::ops::{Add, Sub, Mul, Div};

use super::state_trait::StateVector;
use crate::infrastructure::logger::loggable_trait::Loggable;

// 位置・速度の状態量
#[derive(Debug, Clone)]
pub struct PositionVelocityStateEci {
    state: Array1<f64>, // [p0, p1, p2, v0, v1, v2]
}

impl PositionVelocityStateEci {
    pub fn form_from_list(position: [f64; 3], velocity: [f64; 3]) -> Self {
        let state = arr1(&[position[0], position[1], position[2], velocity[0], velocity[1], velocity[2]]);
        Self { state }
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

impl StateVector for PositionVelocityStateEci {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { state: vec }
    }
}

impl Loggable for PositionVelocityStateEci{
    fn output_log(&self) -> String {
        let state_str : Vec<String> = self.get_vector().iter().map(|v| v.to_string()).collect();
        state_str.join(",")
    }

    fn header(&self) -> String {
        "p0,p1,p2,v0,v1,v2".to_string()
    }
}

/// **演算子のオーバーロード**
impl Add for PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn add(self, rhs: PositionVelocityStateEci) -> PositionVelocityStateEci {
        self.add_vec(&rhs)
    }
}

impl Add for &PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn add(self, rhs: &PositionVelocityStateEci) -> PositionVelocityStateEci {
        self.add_vec(&rhs)
    }
}

impl Sub for PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn sub(self, rhs: PositionVelocityStateEci) -> PositionVelocityStateEci {
        self.sub_vec(&rhs)
    }
}

impl Sub for &PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn sub(self, rhs: &PositionVelocityStateEci) -> PositionVelocityStateEci {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn mul(self, scalar: f64) -> PositionVelocityStateEci {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn mul(self, scalar: f64) -> PositionVelocityStateEci {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn div(self, scalar: f64) -> PositionVelocityStateEci {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &PositionVelocityStateEci {
    type Output = PositionVelocityStateEci;
    fn div(self, scalar: f64) -> PositionVelocityStateEci {
        self.div_scalar(scalar)
    }
}

impl Mul<PositionVelocityStateEci> for Array2<f64> {
    type Output = PositionVelocityStateEci;
    fn mul(self, rhs: PositionVelocityStateEci) -> PositionVelocityStateEci {
        let result = self.dot(rhs.get_vector());
        PositionVelocityStateEci::form_from_array(result)
    }
}

#[cfg(test)]
use ndarray::arr2;
/// **`PositionVelocityStateEci` の基本演算テスト**
#[test]
fn test_position_velocity_state_operations() {
    let pv_state1 = PositionVelocityStateEci::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);
    let pv_state2 = PositionVelocityStateEci::form_from_list([1000.0, 0.0, 0.0], [0.0, -1.5, 0.0]);

    // 加算
    let sum1 = pv_state1.clone() + pv_state2.clone();
    assert_eq!(sum1.get_vector(), &arr1(&[8000.0, 0.0, 0.0, 0.0, 6.0, 0.0]));
    let sum2 = &pv_state1 + &pv_state2;
    assert_eq!(sum2.get_vector(), &arr1(&[8000.0, 0.0, 0.0, 0.0, 6.0, 0.0]));

    // 減算
    let diff = pv_state1.clone() - pv_state2.clone();
    assert_eq!(diff.get_vector(), &arr1(&[6000.0, 0.0, 0.0, 0.0, 9.0, 0.0]));
    let diff2 = &pv_state1 - &pv_state2;
    assert_eq!(diff2.get_vector(), &arr1(&[6000.0, 0.0, 0.0, 0.0, 9.0, 0.0]));

    // スカラー乗算
    let scaled = pv_state1.clone() * 2.0;
    assert_eq!(scaled.get_vector(), &arr1(&[14000.0, 0.0, 0.0, 0.0, 15.0, 0.0]));
    let scaled2 = &pv_state1 * 2.0;
    assert_eq!(scaled2.get_vector(), &arr1(&[14000.0, 0.0, 0.0, 0.0, 15.0, 0.0]));

    // スカラー除算
    let divided = pv_state1.clone() / 2.0;
    assert_eq!(divided.get_vector(), &arr1(&[3500.0, 0.0, 0.0, 0.0, 3.75, 0.0]));
    let divided2 = &pv_state1 / 2.0;
    assert_eq!(divided2.get_vector(), &arr1(&[3500.0, 0.0, 0.0, 0.0, 3.75, 0.0]));
}


/// **行列 x `PositionVelocityStateEci` の変換テスト**
#[test]
fn test_position_velocity_state_matrix_multiplication() {
    let pv_state = PositionVelocityStateEci::form_from_list([7000.0, 0.0, 0.0], [0.0, 7.5, 0.0]);

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