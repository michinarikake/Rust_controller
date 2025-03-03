use ndarray::{Array1, Array2};

use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;

pub trait Differentiable2d<T: StateVector, U: Force> {
    fn differentiate(&self, x: &T, v: &U, t: f64) -> Array2<f64>;
    fn differentiate_numeric(&self, func: &dyn Fn(&T, &U) -> f64, x: &T, v: &U) -> T {
        let epsilon = 1e-5;
        let perturb = x.mul_scalar(epsilon);
        let grad = (func(&(x.add_vec(&perturb)), v) - func(&(x.sub_vec(&perturb)), v)) / (2.0 * epsilon);
        x.mul_scalar(grad) // 適切な変換を行う
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
