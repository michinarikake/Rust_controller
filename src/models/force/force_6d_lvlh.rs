use ndarray::{Array1, Array2, arr1, s};
use std::ops::{Add, Sub, Mul, Div};

use super::force_trait::Force;
use crate::repositry::loggable_trait::Loggable;

#[derive(Debug, Clone)]
pub struct Force6dLvlh{
    force: Array1<f64>
}

impl Force6dLvlh{
    pub fn form_from_list(force_list: [f64;6]) -> Self {
        let force = arr1(
            &[force_list[0], 
            force_list[1], 
            force_list[2],
            force_list[3], 
            force_list[4], 
            force_list[5]
            ]
        );
        Self {force}
    }

    pub fn chief(&self) -> Array1<f64> {
        self.force.slice(s![0..3]).to_owned()
    }

    pub fn deputy(&self) -> Array1<f64> {
        self.force.slice(s![3..6]).to_owned()
    }
}

impl Force for Force6dLvlh {
    fn get_vector(&self) -> &Array1<f64> {
        &self.force
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { force: vec }
    }

    fn zeros() -> Self {
        Self { force: arr1(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) }
    }
}

impl Loggable for Force6dLvlh {
    fn header(&self) -> String {
        "f0,f1,f2,f3,f4,f5".to_string()
    }

    fn output_log(&self) -> String {
        let force_str: Vec<String> = self.get_vector().iter().map(|v| v.to_string()).collect();
        force_str.join(",")
    }
}

/// **演算子のオーバーロード**
impl Add for Force6dLvlh {
    type Output = Force6dLvlh;
    fn add(self, rhs: Force6dLvlh) -> Force6dLvlh {
        self.add_vec(&rhs)
    }
}

impl Add for &Force6dLvlh {
    type Output = Force6dLvlh;
    fn add(self, rhs: &Force6dLvlh) -> Force6dLvlh {
        self.add_vec(&rhs)
    }
}

impl Sub for Force6dLvlh {
    type Output = Force6dLvlh;
    fn sub(self, rhs: Force6dLvlh) -> Force6dLvlh {
        self.sub_vec(&rhs)
    }
}

impl Sub for &Force6dLvlh {
    type Output = Force6dLvlh;
    fn sub(self, rhs: &Force6dLvlh) -> Force6dLvlh {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for Force6dLvlh {
    type Output = Force6dLvlh;
    fn mul(self, scalar: f64) -> Force6dLvlh {
        self.mul_scalar(scalar)
    }
}

impl Mul<f64> for &Force6dLvlh {
    type Output = Force6dLvlh;
    fn mul(self, scalar: f64) -> Force6dLvlh {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for Force6dLvlh {
    type Output = Force6dLvlh;
    fn div(self, scalar: f64) -> Force6dLvlh {
        self.div_scalar(scalar)
    }
}

impl Div<f64> for &Force6dLvlh {
    type Output = Force6dLvlh;
    fn div(self, scalar: f64) -> Force6dLvlh {
        self.div_scalar(scalar)
    }
}

impl Mul<Force6dLvlh> for Array2<f64> {
    type Output = Force6dLvlh;
    fn mul(self, rhs: Force6dLvlh) -> Force6dLvlh {
        let result = self.dot(rhs.get_vector());
        Force6dLvlh::form_from_array(result)
    }
}

impl Mul<&Force6dLvlh> for Array2<f64> {
    type Output = Force6dLvlh;
    fn mul(self, rhs: &Force6dLvlh) -> Force6dLvlh {
        let result = self.dot(rhs.get_vector());
        Force6dLvlh::form_from_array(result)
    }
}