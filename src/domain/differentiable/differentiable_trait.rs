use std::marker::PhantomData;
use std::ops::{Div, Sub};

use ndarray::{Array1, Array2};

use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;

pub trait Differentiable2d<T: StateVector, U: Force> {
    fn differentiate(&self, x: &T, v: &U, t: f64) -> Array2<f64>;
}

pub struct NumericalDefferential<T,U>{
    _marker: PhantomData<T>,
    _marker2: PhantomData<U>
}

impl<T: StateVector, U: Force> NumericalDefferential<T, U>{
    pub fn differentiate_numeric(func: &dyn Fn(&T, &U) -> T, x: &T, v: &U) -> Array2<f64>
    where
        T: StateVector + Sub<T, Output = T> + Div<f64, Output = T>,
    {
        let epsilon = 1e-5;
        let dim = x.get_vector().len(); // 状態ベクトルの次元を取得
        let mut numerical_jacobian = Array2::<f64>::zeros((dim, dim));
        let f_original = func(x, v); // f(z) の値を取得

        for j in 0..dim {
            let mut perturbed_z = x.get_vector().clone();
            perturbed_z[j] += epsilon; // 状態を少しずらす
            let new_state = T::form_from_array(perturbed_z); // z + ε に対応する状態量を生成

            let f_perturbed = func(&new_state, v); // f(z + ε) を計算
            let df = (f_perturbed - f_original.clone()) / epsilon; // 数値微分

            numerical_jacobian.column_mut(j).assign(&df.get_vector()); // 列に格納
        }

        numerical_jacobian
    }

    pub fn differential_numeric_scalar(func: &dyn Fn(&T, &U) -> f64, x: &T, v: &U) -> Array1<f64>
    where
        T: StateVector + Sub<T, Output = T> + Div<f64, Output = T>,
    {
        let epsilon = 1e-5;
        let dim = x.get_vector().len();
        let mut gradient = Array1::<f64>::zeros(dim);

        for i in 0..dim {
            let mut perturbed_x_plus = x.get_vector().clone();
            let mut perturbed_x_minus = x.get_vector().clone();
            perturbed_x_plus[i] += epsilon;
            perturbed_x_minus[i] -= epsilon;

            let new_state_plus = T::form_from_array(perturbed_x_plus);
            let new_state_minus = T::form_from_array(perturbed_x_minus);

            let f_plus = func(&new_state_plus, v);
            let f_minus = func(&new_state_minus, v);

            gradient[i] = (f_plus - f_minus) / (2.0 * epsilon);
        }

        gradient
    }
}


pub trait Differentiable1d<T: StateVector, U: Force> {
    fn differentiate(&self, x: &T, v: &U, t: f64) -> Array1<f64>;
    fn differentiate_numeric(&self, func: &dyn Fn(&T, &U) -> f64, x: &T, v: &U) -> f64 {
        let epsilon = 1e-5;
        let perturb = x.mul_scalar(epsilon);
        let grad = (func(&(x.add_vec(&perturb)), v) - func(&(x.sub_vec(&perturb)), v)) / (2.0 * epsilon);
        grad
    }
}
