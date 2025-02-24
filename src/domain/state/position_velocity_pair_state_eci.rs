use ndarray::{Array1, Array2, arr1, s};
use std::ops::{Add, Sub, Mul, Div};

use crate::domain::state::state_trait::StateVector;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::repositry::loggable_trait::Loggable;
use crate::domain::math::formulations::Math;

use super::orbital_elements::OrbitalElements;

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

    pub fn form_from_state(chief_state: PositionVelocityStateEci, deputy_state: PositionVelocityStateEci) -> Self {
        let chief = chief_state.get_vector();
        let deputy = deputy_state.get_vector();
        let state = arr1(
            &[chief[0], chief[1], chief[2], chief[3], chief[4], chief[5],
             deputy[0], deputy[1], deputy[2], deputy[3], deputy[4], deputy[5]]
            );
        Self { state }
    }

    pub fn form_from_orbital_elements(chief_oe: OrbitalElements, deputy_oe: OrbitalElements, mu: f64) -> Self {
        let chief = PositionVelocityStateEci::form_from_orbital_elements(&chief_oe, mu);
        let deputy = PositionVelocityStateEci::form_from_orbital_elements(&deputy_oe, mu);
        PositionVelocityPairStateEci::form_from_state(chief, deputy)
    }

    pub fn chief(&self) -> PositionVelocityStateEci {
        PositionVelocityStateEci::form_from_array(self.state.slice(s![0..6]).to_owned())
    }

    pub fn deputy(&self) -> PositionVelocityStateEci {
        PositionVelocityStateEci::form_from_array(self.state.slice(s![6..12]).to_owned())
    }

    pub fn relative_position(&self) -> PositionVelocityStateLvlh {
        PositionVelocityStateLvlh::form_from_array(
            Math::convert_to_lvlh(self.chief().get_vector(), self.deputy().get_vector())
        )
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
        let state_str : Vec<String> = self.relative_position().get_vector().iter().map(|v| v.to_string()).collect();
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