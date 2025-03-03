use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;

pub trait Cost<T: StateVector, U: Force> {
    fn evaluate(&self, x: &Vec<T>, v: &Vec<U>) -> f64 
    {
        let mut cost = 0.0;
        for i in 0..x.len() {
            cost += self.stage_cost(&x[i], &v[i]);
        }
        cost += self.terminal_cost(&x[x.len() - 1]);
        cost
    }
    fn stage_cost(&self, x: &T, v: &U) -> f64;
    fn terminal_cost(&self, x: &T) -> f64;
}
