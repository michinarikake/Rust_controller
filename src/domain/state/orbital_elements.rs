use ndarray::{Array1, arr1};
use std::ops::{Add, Sub, Mul, Div};
use std::f64::consts::PI;

use super::state_trait::StateVector;
use crate::infrastructure::logger::loggable_trait::Loggable;

#[derive(Debug, Clone)]
#[allow(non_snake_case)]
pub struct OrbitalElements {
    state: Array1<f64>, // [a ,e, i_rad,omega_rad, Omega_rad, nu_rad]
    pub a: f64,      // 長半径 (m)
    pub e: f64,      // 離心率
    pub i_rad: f64,  // 軌道傾斜角 (rad)
    pub omega_rad: f64,  // 近地点引数 (rad)
    pub Omega_rad: f64,  // 昇交点経度 (rad)
    pub nu_rad: f64,     // 真近点角 (rad)
}

impl OrbitalElements {
    #[allow(non_snake_case)]
    pub fn form_from_elements(a: f64, e: f64, i_rad: f64, omega_rad: f64, Omega_rad: f64, nu_rad: f64) -> Result<Self, &'static str> {
        if a <= 0.0 {
            return Err("軌道長半径 (a) は正の値でなければなりません。");
        }
        if e < 0.0 || e >= 1.0 {
            return Err("離心率 (e) は 0 以上 1 未満でなければなりません。");
        }
        if i_rad < 0.0 || i_rad > PI {
            return Err("軌道傾斜角 (i) は 0 以上 π 以下でなければなりません。");
        }
        Ok(Self {state: arr1(&[a, e, i_rad, omega_rad, Omega_rad, nu_rad]), a, e, i_rad, omega_rad, Omega_rad, nu_rad })
    }
}

impl StateVector for OrbitalElements {
    fn get_vector(&self) -> &Array1<f64> {
        &self.state
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { state: vec.clone() , a: vec[0], e: vec[1], i_rad: vec[2], omega_rad: vec[3], Omega_rad: vec[4], nu_rad: vec[5]}
    }
}

impl Loggable for OrbitalElements{
    fn output_log(&self) -> String {
        let state_str : Vec<String> = self.get_vector().iter().map(|v| v.to_string()).collect();
        state_str.join(",")
    }

    fn header(&self) -> String {
        "a,e,i_rad,omega_rad,Omega_rad,nu_rad".to_string()
    }
}


/// **演算子のオーバーロード**
impl Add for OrbitalElements {
    type Output = OrbitalElements;
    fn add(self, rhs: OrbitalElements) -> OrbitalElements {
        self.add_vec(&rhs)
    }
}

impl Add for &OrbitalElements {
    type Output = OrbitalElements;
    fn add(self, rhs: &OrbitalElements) -> OrbitalElements {
        self.add_vec(&rhs)
    }
}

impl Sub for OrbitalElements {
    type Output = OrbitalElements;
    fn sub(self, rhs: OrbitalElements) -> OrbitalElements {
        self.sub_vec(&rhs)
    }
}

impl Sub for &OrbitalElements {
    type Output = OrbitalElements;
    fn sub(self, rhs: &OrbitalElements) -> OrbitalElements {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for OrbitalElements {
    type Output = OrbitalElements;
    fn mul(self, scalar: f64) -> OrbitalElements {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &OrbitalElements {
    type Output = OrbitalElements;
    fn mul(self, scalar: f64) -> OrbitalElements {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for OrbitalElements {
    type Output = OrbitalElements;
    fn div(self, scalar: f64) -> OrbitalElements {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &OrbitalElements {
    type Output = OrbitalElements;
    fn div(self, scalar: f64) -> OrbitalElements {
        self.div_scalar(scalar)
    }
}