use ndarray::{Array1, Array2, arr1, arr2, concatenate, Axis, s};
use std::ops::{Add, Sub, Mul, Div};

use crate::domain::state::state_trait::StateVector;
use crate::domain::state::position_velocity_state_ecef::PositionVelocityStateEcef;
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::repositry::loggable_trait::Loggable;
use crate::domain::math::formulations::Math;

#[derive(Debug, Clone)]
pub struct PositionVelocityPairStateEcef {
    state: Array1<f64>, // [chief_px, chief_py, chief_pz, chief_vx, chief_vy, chief_vz, deputy_px, deputy_py, deputy_pz, deputy_vx, deputy_vy, deputy_vz]
}

impl PositionVelocityPairStateEcef {
    /// **新規作成**
    pub fn form_from_states(chief: &PositionVelocityStateEcef, deputy: &PositionVelocityStateEcef) -> Self {
        let state = concatenate![
            Axis(0),
            chief.get_vector().view(),  // `view()` を追加
            deputy.get_vector().view()  // `view()` を追加
        ];

        Self { state }
    }

    /// **チーフ衛星の状態を取得**
    pub fn chief(&self) -> PositionVelocityStateEcef {
        PositionVelocityStateEcef::form_from_array(self.state.slice(s![0..6]).to_owned())
    }

    /// **デピュティ衛星の状態を取得**
    pub fn deputy(&self) -> PositionVelocityStateEcef {
        PositionVelocityStateEcef::form_from_array(self.state.slice(s![6..12]).to_owned())
    }

    pub fn relative_position(&self) -> PositionVelocityStateLvlh {
        PositionVelocityStateLvlh::form_from_array(
            Math::convert_to_lvlh(self.chief().get_vector(), self.deputy().get_vector())
        )
    }

}

impl StateVector for PositionVelocityPairStateEcef {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { state: vec }
    }
}

impl Loggable for PositionVelocityPairStateEcef{
    fn output_log(&self) -> String {
        let state_str : Vec<String> = self.relative_position().get_vector().iter().map(|v| v.to_string()).collect();
        state_str.join(",")
    }

    fn header(&self) -> String {
        "p0,p1,p2,v0,v1,v2".to_string()
    }
}

/// **演算子のオーバーロード**
impl Add for PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn add(self, rhs: PositionVelocityPairStateEcef) -> PositionVelocityPairStateEcef {
        self.add_vec(&rhs)
    }
}

impl Add for &PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn add(self, rhs: &PositionVelocityPairStateEcef) -> PositionVelocityPairStateEcef {
        self.add_vec(&rhs)
    }
}

impl Sub for PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn sub(self, rhs: PositionVelocityPairStateEcef) -> PositionVelocityPairStateEcef {
        self.sub_vec(&rhs)
    }
}

impl Sub for &PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn sub(self, rhs: &PositionVelocityPairStateEcef) -> PositionVelocityPairStateEcef {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn mul(self, scalar: f64) -> PositionVelocityPairStateEcef {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn mul(self, scalar: f64) -> PositionVelocityPairStateEcef {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn div(self, scalar: f64) -> PositionVelocityPairStateEcef {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &PositionVelocityPairStateEcef {
    type Output = PositionVelocityPairStateEcef;
    fn div(self, scalar: f64) -> PositionVelocityPairStateEcef {
        self.div_scalar(scalar)
    }
}

impl Mul<PositionVelocityPairStateEcef> for Array2<f64> {
    type Output = PositionVelocityPairStateEcef;
    fn mul(self, rhs: PositionVelocityPairStateEcef) -> PositionVelocityPairStateEcef {
        let result = self.dot(rhs.get_vector());
        PositionVelocityPairStateEcef::form_from_array(result)
    }
}