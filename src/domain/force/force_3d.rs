use ndarray::{Array1, Array2, arr1};
use std::ops::{Add, Sub, Mul, Div};

use super::force_trait::Force;
use crate::repositry::loggable_trait::Loggable;

#[derive(Debug, Clone)]
pub struct Force3D{
    force: Array1<f64>
}

impl Force3D{
    pub fn form_from_list(force_list: [f64;3]) -> Self {
        let force = arr1(&[force_list[0], force_list[1], force_list[2]]);
        Self {force}
    }
}

impl Force for Force3D {
    fn get_vector(&self) -> &Array1<f64> {
        &self.force
    }

    fn form_from_array(vec: Array1<f64>) -> Self {
        Self { force: vec }
    }
}

impl Loggable for Force3D {
    fn header(&self) -> String {
        "f0,f1,f2".to_string()
    }

    fn output_log(&self) -> String {
        let force_str: Vec<String> = self.get_vector().iter().map(|v| v.to_string()).collect();
        force_str.join(",")
    }
}

/// **演算子のオーバーロード**
impl Add for Force3D {
    type Output = Force3D;
    fn add(self, rhs: Force3D) -> Force3D {
        self.add_vec(&rhs)
    }
}

impl Sub for Force3D {
    type Output = Force3D;
    fn sub(self, rhs: Force3D) -> Force3D {
        self.sub_vec(&rhs)
    }
}

impl Mul<f64> for Force3D {
    type Output = Force3D;
    fn mul(self, scalar: f64) -> Force3D {
        self.mul_scalar(scalar)
    }
}

impl Div<f64> for Force3D {
    type Output = Force3D;
    fn div(self, scalar: f64) -> Force3D {
        self.div_scalar(scalar)
    }
}

impl Mul<Force3D> for Array2<f64> {
    type Output = Force3D;
    fn mul(self, rhs: Force3D) -> Force3D {
        let result = self.dot(rhs.get_vector());
        Force3D::form_from_array(result)
    }
}

use ndarray::{arr2};

#[test]
fn test_force3d_operations() {
    let f1 = Force3D::form_from_list([1.0, 2.0, 3.0]);
    let f2 = Force3D::form_from_list([0.5, -1.0, 2.0]);

    // 加算
    let sum = f1.clone() + f2.clone();
    assert_eq!(sum.get_vector(), &arr1(&[1.5, 1.0, 5.0]));

    // 減算
    let diff = f1.clone() - f2.clone();
    assert_eq!(diff.get_vector(), &arr1(&[0.5, 3.0, 1.0]));

    // スカラー乗算
    let scaled = f1.clone() * 2.0;
    assert_eq!(scaled.get_vector(), &arr1(&[2.0, 4.0, 6.0]));

    // スカラー除算
    let divided = f1.clone() / 2.0;
    assert_eq!(divided.get_vector(), &arr1(&[0.5, 1.0, 1.5]));
}

#[test]
fn test_force3d_matrix_multiplication() {
    let f = Force3D::form_from_list([1.0, 2.0, 3.0]);

    let transform_matrix = arr2(&[
        [2.0, 0.0, 0.0],
        [0.0, 3.0, 0.0],
        [0.0, 0.0, 4.0],
    ]);

    let transformed = transform_matrix * f.clone();
    assert_eq!(transformed.get_vector(), &arr1(&[2.0, 6.0, 12.0]));
}
