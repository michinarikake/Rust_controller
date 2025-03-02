use ndarray::{Array1, Array2};
use std::collections::HashMap;
use crate::domain::differentiable::differentiable_trait::Differentiable;
use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::cost::cost_trait::Cost;
use crate::domain::dynamics::propagator::Propagator;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;

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

/// **評価関数のトレイト**
pub trait EvaluationFunction {
    fn evaluate(&self, schedule: &ModeSchedule) -> f64;
    fn gradient(&self, schedule: &ModeSchedule) -> Array1<f64>;
}

/// **モード遷移最適化用の評価関数**
pub struct ModeTransitionEvaluator;

impl EvaluationFunction for ModeTransitionEvaluator {
    fn evaluate(&self, _schedule: &ModeSchedule) -> f64 {
        // 仮の実装
        0.0
    }

    fn gradient(&self, _schedule: &ModeSchedule) -> Array1<f64> {
        // 空の実装
        Array1::zeros(1)
    }
}

/// **Insertion Gradient の計算を担当する構造体**
pub struct InsertionGradientCalculator;

impl InsertionGradientCalculator {
    pub fn compute<T: StateVector, U: Force, D: ContinuousDynamics<T, U>>(
        &self,
        schedule: &ModeSchedule,
        dynamics: &D,
        propagator: &dyn Propagator<T, U>,
        dt: f64,
    ) -> Array2<f64> {
        let num_times = 10; // 仮の時刻数
        let num_modes = 5; // 仮のモード数
        let mut d = Array2::zeros((num_times, num_modes));

        for t in 0..num_times {
            for m in 0..num_modes {
                let prev_mode = schedule.get(t).copied().unwrap_or(0);
                let x_prev = propagator.propagate_continuous(&schedule, prev_mode, dynamics, dt);
                let x_new = propagator.propagate_continuous(&schedule, m, dynamics, dt);
                let p = AdjointVariableUpdater::compute_p(&x_prev, &x_new);
                d[(t, m)] = p.dot(&(x_new.get_vector() - x_prev.get_vector()));
            }
        }
        d
    }
}

/// **随伴変数の更新（逆伝播）を担当**
pub struct AdjointVariableUpdater;

impl AdjointVariableUpdater {
    pub fn update(&mut self) {
        // 空の実装
    }

    pub fn compute_p<T: StateVector>(x_prev: &T, x_new: &T) -> Array1<f64> {
        let gradient = x_new.get_vector() - x_prev.get_vector();
        -gradient // 仮の逆伝播計算
    }
}

/// **Armijo 条件を適用してモードスケジュールを更新**
pub struct ArmijoOptimizer {
    pub eta: f64,
    pub alpha: f64,
    pub beta: f64,
    pub lambda: f64,
}

impl ArmijoOptimizer {
    pub fn apply(&mut self, d: &Array2<f64>, evaluator: &dyn EvaluationFunction, schedule: &ModeSchedule) -> ModeSchedule {
        let d_min = d.iter().cloned().fold(f64::INFINITY, f64::min);
        let threshold = self.eta * d_min;

        let mut selected_times = Vec::new();
        let mut selected_modes = Vec::new();
        
        for (t, row) in d.axis_iter(ndarray::Axis(0)).enumerate() {
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
            if measure > self.lambda {
                break;
            }
            subset_times.push(t);
            subset_modes.push(m);
        }

        let mut new_schedule = schedule.clone();
        for (&t, &m) in subset_times.iter().zip(subset_modes.iter()) {
            new_schedule.update(t, m);
        }

        let new_value = evaluator.evaluate(&new_schedule);
        let old_value = evaluator.evaluate(schedule);
        
        if new_value - old_value <= self.alpha * self.lambda * d_min {
            return new_schedule;
        } else {
            self.lambda *= self.beta;
            return self.apply(d, evaluator, schedule);
        }
    }
}

/// **モードスケジューラ（メインの最適化ループ）**
pub struct ModeScheduler {
    max_iterations: usize,
}

impl ModeScheduler {
    pub fn new(max_iterations: usize) -> Self {
        Self { max_iterations }
    }

    pub fn optimize<T, U, D>(&mut self, dynamics: &D, propagator: &dyn Propagator<T, U>, evaluator: &dyn EvaluationFunction) -> ModeSchedule
    where
        T: StateVector,
        U: Force,
        D: ContinuousDynamics<T, U> + Differentiable<T, U>,
    {
        let mut schedule = ModeSchedule::new(HashMap::new());
        let gradient_calculator = InsertionGradientCalculator;
        let mut armijo_optimizer = ArmijoOptimizer {
            eta: 0.1,
            alpha: 0.5,
            beta: 0.9,
            lambda: 1.0,
        };
        let mut adjoint_updater = AdjointVariableUpdater;

        for _ in 0..self.max_iterations {
            let d = gradient_calculator.compute(&schedule, dynamics, propagator, 1.0);
            let new_schedule = armijo_optimizer.apply(&d, evaluator, &schedule);
            let new_value = evaluator.evaluate(&new_schedule);
            let old_value = evaluator.evaluate(&schedule);

            if new_value - old_value <= 0.01 {
                return new_schedule;
            }
            schedule = new_schedule;
            adjoint_updater.update();
        }
        schedule
    }
}