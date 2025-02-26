use ndarray::{Array1, Array2, arr1, s};
use std::ops::{Add, Sub, Mul, Div};

use crate::domain::state::state_trait::StateVector;
use crate::infrastructure::logger::loggable_trait::Loggable;
// FIXME ここのincludeは本来不要
// FIXME LOGGERを分離する
use super::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use super::state_converter::StateConverter;

#[derive(Debug, Clone)]
pub struct PositionVelocityPairStateEci {
    state: Array1<f64>, // [chief_px, chief_py, chief_pz, chief_vx, chief_vy, chief_vz, 
                        // deputy_px, deputy_py, deputy_pz, deputy_vx, deputy_vy, deputy_vz]
}

impl PositionVelocityPairStateEci {

    pub fn form_from_list(chief: [f64; 6], deputy: [f64; 6]) -> Self {
        let state = arr1(
            &[chief[0], chief[1], chief[2], chief[3], chief[4], chief[5],
             deputy[0], deputy[1], deputy[2], deputy[3], deputy[4], deputy[5]]
            );
        Self { state }
    }

    pub fn chief(&self) -> Array1<f64> {
        self.state.slice(s![0..6]).to_owned()
    }

    pub fn deputy(&self) -> Array1<f64> {
        self.state.slice(s![6..12]).to_owned()
    }

}

impl StateVector for PositionVelocityPairStateEci {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { state: vec }
    }
}

impl Loggable for PositionVelocityPairStateEci{
    fn output_log(&self) -> String {
        let state_vec: PositionVelocityStateLvlh = self.convert();
        let state_str : Vec<String> = state_vec.get_vector().iter().map(|v| v.to_string()).collect();
        state_str.join(",")
    }

    fn header(&self) -> String {
        "p0,p1,p2,v0,v1,v2".to_string()
    }
}

/// **演算子のオーバーロード**
impl Add for PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn add(self, rhs: PositionVelocityPairStateEci) -> PositionVelocityPairStateEci {
        self.add_vec(&rhs)
    }
}

impl Add for &PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn add(self, rhs: &PositionVelocityPairStateEci) -> PositionVelocityPairStateEci {
        self.add_vec(&rhs)
    }
}

impl Sub for PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn sub(self, rhs: PositionVelocityPairStateEci) -> PositionVelocityPairStateEci {
        self.sub_vec(&rhs)
    }
}

impl Sub for &PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn sub(self, rhs: &PositionVelocityPairStateEci) -> PositionVelocityPairStateEci {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn mul(self, scalar: f64) -> PositionVelocityPairStateEci {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn mul(self, scalar: f64) -> PositionVelocityPairStateEci {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn div(self, scalar: f64) -> PositionVelocityPairStateEci {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &PositionVelocityPairStateEci {
    type Output = PositionVelocityPairStateEci;
    fn div(self, scalar: f64) -> PositionVelocityPairStateEci {
        self.div_scalar(scalar)
    }
}

impl Mul<PositionVelocityPairStateEci> for Array2<f64> {
    type Output = PositionVelocityPairStateEci;
    fn mul(self, rhs: PositionVelocityPairStateEci) -> PositionVelocityPairStateEci {
        let result = self.dot(rhs.get_vector());
        PositionVelocityPairStateEci::form_from_array(result)
    }
}