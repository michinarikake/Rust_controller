use ndarray::{Array1, Array2};
use std::collections::HashMap;
use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::cost::cost_trait::Cost;
use crate::domain::dynamics::propagator::Propagator;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::differentiable::differentiable_trait::Differentiable;

/// **モードとダイナミクスの対応関係を表現する構造体**
pub struct ModeDynamicsMap<T, U, D>
where
    T: StateVector,
    U: Force,
    D: ContinuousDynamics<T, U>,
{
    mapping: HashMap<usize, D>, // モード -> ダイナミクス
}

impl<T, U, D> ModeDynamicsMap<T, U, D>
where
    T: StateVector,
    U: Force,
    D: ContinuousDynamics<T, U>,
{
    pub fn new(mapping: HashMap<usize, D>) -> Self {
        Self { mapping }
    }

    pub fn get_dynamics(&self, mode: usize) -> Option<&D> {
        self.mapping.get(&mode)
    }
}

/// **モードスケジュールを表現する構造体**
pub struct ModeSchedule {
    schedule: HashMap<usize, usize>, // 時刻 -> モード
}

impl ModeSchedule {
    pub fn new(schedule: HashMap<usize, usize>) -> Self {
        Self { schedule }
    }

    pub fn get(&self, time: usize) -> Option<&usize> {
        self.schedule.get(&time)
    }

    pub fn update(&mut self, time: usize, mode: usize) {
        self.schedule.insert(time, mode);
    }
}

/// **状態量の更新（各時刻のモードに対応するダイナミクスから微分値を求める）**
pub fn update_states<T, U, D, P>(
    schedule: &ModeSchedule,
    mode_dynamics_map: &ModeDynamicsMap<T, U, D>,
    propagator: &P,
    dt: f64,
) -> Vec<T>
where
    T: StateVector,
    U: Force,
    D: ContinuousDynamics<T, U>,
    P: Propagator<T, U>,
{
    let mut states = vec![];
    for t in 0..schedule.schedule.len() {
        if let Some(&mode) = schedule.get(t) {
            if let Some(dynamics) = mode_dynamics_map.get_dynamics(mode) {
                let prev_state = if t == 0 {
                    T::form_from_array(Array1::zeros(3)) // 初期状態（仮）
                } else {
                    states[t - 1].clone()
                };
                let new_state = propagator.propagate_continuous(&prev_state, &U::form_from_array(Array1::zeros(prev_state.get_vector().len())), dynamics, dt);
                states.push(new_state);
            }
        }
    }
    states
}

/// **Armijo 条件を適用してモードスケジュールを更新**
pub struct ArmijoOptimizer {
    pub eta: f64,
    pub alpha: f64,
    pub beta: f64,
    pub lambda: f64,
}

impl ArmijoOptimizer {
    pub fn apply<T, U, D, P>(
        &mut self,
        D: &Array2<f64>,
        evaluator: &dyn Cost<T, U>,
        schedule: &ModeSchedule,
        mode_dynamics_map: &ModeDynamicsMap<T, U, D>,
        propagator: &P,
        dt: f64,
    ) -> ModeSchedule
    where
        T: StateVector,
        U: Force,
        D: ContinuousDynamics<T, U>,
        P: Propagator<T, U>,
    {
        let D_min = D.iter().cloned().fold(f64::INFINITY, f64::min);
        let mut lambda = self.lambda;
        
        loop {
            let threshold = self.eta * D_min;
            let mut selected_times = Vec::new();
            let mut selected_modes = Vec::new();
            
            for (t, row) in D.axis_iter(ndarray::Axis(0)).enumerate() {
                if let Some((min_index, &min_value)) = row.iter().enumerate().min_by(|a, b| a.1.partial_cmp(b.1).unwrap()) {
                    if min_value <= threshold {
                        selected_times.push(t);
                        selected_modes.push(min_index);
                    }
                }
            }

            let mut subset_times = Vec::new();
            let mut subset_modes = Vec::new();
            let mut measure = 0.0;

            for (&t, &m) in selected_times.iter().zip(selected_modes.iter()) {
                measure += 1.0;
                if measure > lambda {
                    break;
                }
                subset_times.push(t);
                subset_modes.push(m);
            }

            let mut new_schedule = schedule.clone();
            for (&t, &m) in subset_times.iter().zip(subset_modes.iter()) {
                new_schedule.update(t, m);
            }
            
            let states = update_states(&new_schedule, mode_dynamics_map, propagator, dt);
            
            let new_value = evaluator.evaluate(&new_schedule, &states);
            let old_value = evaluator.evaluate(schedule, &states);
            
            if new_value - old_value <= self.alpha * lambda * D_min {
                return new_schedule;
            } else {
                lambda *= self.beta;
            }
        }
    }
}
