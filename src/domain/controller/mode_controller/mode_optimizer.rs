use ndarray::{Array1, Array2};
use std::collections::HashMap;
use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::cost::cost_trait::Cost;
use crate::domain::dynamics::propagator::Propagator;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::differentiable::differentiable_trait::{Differentiable2d, Differentiable1d};

#[derive(PartialEq, Eq, Hash, Clone, Copy)]
pub struct ModeId{
    id: usize,
}

impl ModeId {
    pub fn new(id: usize) -> Self {
        Self { id }
    }
}

pub trait ContinuousDynamicsAndDifferentiable<T, U>: ContinuousDynamics<T, U> + Differentiable2d<T, U>
where
    T: StateVector,
    U: Force,
{
    fn as_continuous_dynamics(&self) -> &dyn ContinuousDynamics<T, U>;
}

impl<T, U, D> ContinuousDynamicsAndDifferentiable<T, U> for D
where
    T: StateVector,
    U: Force,
    D: ContinuousDynamics<T, U> + Differentiable2d<T, U>,
{
    fn as_continuous_dynamics(&self) -> &dyn ContinuousDynamics<T, U> {
        self
    }
}


pub trait CostAndDifferentiable<T, U>: Cost<T, U> + Differentiable1d<T, U> 
where
    T: StateVector,
    U: Force,
{}


/// **モードとダイナミクスの対応関係を表現する構造体**
pub struct ModeDynamicsMap<T, U>
where
    T: StateVector,
    U: Force,
{
    dynamics_mapping: HashMap<ModeId, Box<dyn ContinuousDynamicsAndDifferentiable<T, U>>>, // モード -> ダイナミクス
    cost_mapping: HashMap<ModeId, Box<dyn CostAndDifferentiable<T, U>>>, // モード -> 評価関数
}

impl<T, U> ModeDynamicsMap<T, U>
where
    T: StateVector,
    U: Force,
{
    pub fn new
    (
        dynamics_mapping: HashMap<ModeId, Box<dyn ContinuousDynamicsAndDifferentiable<T, U>>>, 
        cost_mapping: HashMap<ModeId, Box<dyn CostAndDifferentiable<T, U>>>
    ) -> Self 
    {
        Self { dynamics_mapping, cost_mapping }
    }

    pub fn get_dynamics(&self, mode: ModeId) -> Option<&dyn ContinuousDynamicsAndDifferentiable<T, U>> {
        self.dynamics_mapping.get(&mode).map(|d| d.as_ref())
    }

    pub fn get_cost(&self, mode: ModeId) -> Option<&dyn CostAndDifferentiable<T, U>> {
        self.cost_mapping.get(&mode).map(|c| c.as_ref())
    }
}

/// **モードスケジュールを表現する構造体**
#[derive(Clone)]
pub struct ModeSchedule {
    schedule: HashMap<usize, ModeId>, // 時刻 -> モード
}

impl ModeSchedule {
    pub fn new(t_index0: usize, t_index_last: usize, schedule: &HashMap<usize, ModeId>) -> Self {
        for t in t_index0..t_index_last {
            if !schedule.contains_key(&t) {
                panic!("Schedule does not contain key {}", t);
            }
        }
        Self { schedule: schedule.clone() }
    }

    pub fn new_from_mode0(t_index0: usize, t_index_last: usize, mode0: ModeId) -> Self {
        let mut schedule = HashMap::new();
        for t in t_index0..t_index_last {
            schedule.insert(t, mode0);
        }
        Self { schedule }
    }

    pub fn get(&self, time: usize) -> Option<&ModeId> {
        self.schedule.get(&time)
    }

    pub fn insert(&mut self, time: usize, mode: ModeId) {
        self.schedule.insert(time, mode);
    }
}

pub struct StateSchedule<T: StateVector> {
    state_schedule: HashMap<usize, T>,
}

impl<T: StateVector> StateSchedule<T> {
    pub fn new(t_index0: usize, t_index_last: usize, state_schedule: &HashMap<usize, T>) -> Self {
        for t in t_index0..t_index_last {
            if !state_schedule.contains_key(&t) {
                panic!("Schedule does not contain key {}", t);
            }
        }
        Self { state_schedule: state_schedule.clone() }
    }

    pub fn get(&self, time: usize) -> Option<&T> {
        self.state_schedule.get(&time)
    }

    pub fn to_vec(&self) -> Vec<T> {
        let mut vec = Vec::new();
        for t in 0..self.state_schedule.len() {
            vec.push(self.state_schedule[&t].clone());
        }
        vec
    }
}

pub struct AdjointVariableSchedule<T: StateVector> {
    adjoint_schedule: HashMap<usize, T>,
}

impl<T: StateVector> AdjointVariableSchedule<T> {
    pub fn new(t_index0: usize, t_index_last: usize, adjoint_schedule: &HashMap<usize, T>) -> Self {
        for t in t_index0..t_index_last {
            if !adjoint_schedule.contains_key(&t) {
                panic!("Schedule does not contain key {}", t);
            }
        }
        Self { adjoint_schedule: adjoint_schedule.clone() }
    }

    pub fn get(&self, time: usize) -> Option<&T> {
        self.adjoint_schedule.get(&time)
    }

    pub fn to_vec(&self) -> Vec<T> {
        let mut vec = Vec::new();
        for t in 0..self.adjoint_schedule.len() {
            vec.push(self.adjoint_schedule[&t].clone());
        }
        vec
    }
}

pub struct StateUpdater;

impl StateUpdater {
    pub fn new<T, U, P>(
        x0: &T,
        mode_schedule: &ModeSchedule,
        mode_dynamics_map: &ModeDynamicsMap<T, U>,
        propagator: &P,
        dt: f64,
    ) -> StateSchedule<T>
    where
        T: StateVector,
        U: Force,
        P: Propagator<T, U>,
    {
        let mut states: HashMap<usize, T> = HashMap::new();

        for t_index in 0..mode_schedule.schedule.len() {

            let mode = mode_schedule.get(t_index).unwrap();
            let dynamics = mode_dynamics_map.get_dynamics(*mode).unwrap();

            let prev_state = if t_index == 0 {
                x0.clone()
            } else {
                states[&(t_index - 1)].clone()
            };

            let new_state = propagator.propagate_continuous(
                &prev_state,
                &U::form_from_array(Array1::zeros(prev_state.get_vector().len())), 
                dynamics.as_continuous_dynamics(), 
                dt);

            states.insert(t_index, new_state);
        }
        StateSchedule::new(0, mode_schedule.schedule.len(), &states)
    }
}

/// **随伴変数の更新（逆伝播）を担当**
pub struct AdjointVariableUpdater;

impl AdjointVariableUpdater {
    pub fn new<T, U>(
        states: &StateSchedule<T>,
        mode_schedule: &ModeSchedule,
        mode_dynamics_map: &ModeDynamicsMap<T, U>,
        dt: f64,
    ) -> AdjointVariableSchedule<T>
    where
        T: StateVector,
        U: Force,
    {
        let mut p_map: HashMap<usize, T> = HashMap::new();
        let t_index_last = mode_schedule.schedule.len() - 1;

        // 終端コストの勾配が終端時刻の随伴変数
        let mode_last = mode_schedule.get(t_index_last).unwrap();
        let cost_last = mode_dynamics_map.get_cost(*mode_last).unwrap();
        let p_last = T::form_from_array(cost_last.differentiate(&states.get(t_index_last).unwrap(), &U::zeros(),t_index_last as f64 * dt + 1.0));
        p_map.insert(t_index_last, p_last.clone());

        for t_index in (0..t_index_last).rev() {
            let p = p_map[&(t_index + 1)].clone();
            let p_dot = Self::compute_derivative::<T, U>(&p, &states.get(t_index).unwrap(), t_index, mode_schedule, mode_dynamics_map, dt);
            let p_prev = p.add_vec(&p_dot.mul_scalar(-dt));
            p_map.insert(t_index, p_prev);
        }
        AdjointVariableSchedule::new(0, t_index_last, &p_map)
    }

    fn compute_derivative<T, U>(
        p: &T, 
        x_prev: &T, 
        t_index: usize,
        schedule: &ModeSchedule,
        mode_dynamics_map: &ModeDynamicsMap<T, U>,
        dt: f64,
    ) -> T
    where
        T: StateVector,
        U: Force,
    {
        let mode =  schedule.get(t_index).unwrap();
        let dynamics = mode_dynamics_map.get_dynamics(*mode).unwrap();
        let grad = dynamics.differentiate(x_prev, &U::zeros(), t_index as f64 * dt);
        p.mul_mat(&grad)
    }
}

/// **Insertion Gradient の計算を担当する構造体**
pub struct InsertionGradientCalculator;

impl InsertionGradientCalculator {
    pub fn compute<T, U>(
        mode_schedule: &ModeSchedule,
        state_schedule: &StateSchedule<T>,
        adjoint_schedule: &AdjointVariableSchedule<T>,
        mode_dynamics_map: &ModeDynamicsMap<T, U>,
        t_index0: usize,
        t_index_last: usize,
     ) -> Array2<f64>
    where
        T: StateVector,
        U: Force,
    {
        let num_modes = mode_dynamics_map.dynamics_mapping.len();
        let mut d = Array2::zeros((t_index_last, num_modes));

        for t_index in t_index0..t_index_last {
            let mode_prev = mode_schedule.get(t_index).unwrap();
            let dynamics_prev = mode_dynamics_map.get_dynamics(*mode_prev).unwrap();
            let x_now = state_schedule.get(t_index).unwrap();
            let p_now = adjoint_schedule.get(t_index).unwrap();


            for m_index in 0..num_modes {
                let mode_new = ModeId::new(m_index);
                let dynamics_new = mode_dynamics_map.get_dynamics(mode_new).unwrap();
                let x_dot_prev = dynamics_prev.compute_derivative(x_now, &U::zeros());
                let x_dot_new = dynamics_new.compute_derivative(x_now, &U::zeros());
                d[(t_index, m_index)] = Self::compute_insertion_gradient::<T>(&p_now, &x_dot_prev, &x_dot_new)
            }
        }
        d
    }

    pub fn compute_insertion_gradient<T: StateVector>(p: &T, x_dot_prev: &T, x_dot_new: &T) -> f64 {
        p.get_vector().dot(&(x_dot_new.get_vector() - x_dot_prev.get_vector()))
    }
}

/// **Armijo 条件を適用してモードスケジュールを更新**
pub struct ModeScheduler<T, U, P>
where
    T: StateVector,
    U: Force,
    P: Propagator<T, U>,
{
    pub eta: f64,
    pub alpha: f64,
    pub beta: f64,
    pub lambda: f64,
    pub max_iterations: usize,
    pub t_index0: usize,
    pub t_index_last: usize,
    x0: T,
    mode_dynamics_map: ModeDynamicsMap<T, U>,
    propagator: P,
    dt: f64,
}

impl<T, U, P> ModeScheduler<T, U, P> 
where
    T: StateVector,
    U: Force,
    P: Propagator<T, U>,
{
    pub fn new(eta: f64, 
        alpha: f64, 
        beta: f64, 
        lambda: f64, 
        max_iterations: usize, 
        t_index0: usize, 
        t_index_last: usize, 
        x0: T, 
        dynamics_mapping: HashMap<ModeId, Box<dyn ContinuousDynamicsAndDifferentiable<T, U>>>, 
        cost_mapping: HashMap<ModeId, Box<dyn CostAndDifferentiable<T, U>>>,
        propagator: P,
        dt: f64,
    ) -> Self {
        let mode_dynamics_map = ModeDynamicsMap::new(dynamics_mapping, cost_mapping);
        Self { eta, alpha, beta, lambda , max_iterations, t_index0, t_index_last, x0: x0.clone(), 
            mode_dynamics_map, propagator, dt }
    }

    pub fn optimize(&mut self) -> ModeSchedule
    {
        let mut mode_schedule = ModeSchedule::new_from_mode0(self.t_index0, self.t_index_last, ModeId::new(0));
        let mut state_schedule = StateUpdater::new(&self.x0, &mode_schedule, &self.mode_dynamics_map, &self.propagator, self.dt);
        let mut adjoint_schedule = AdjointVariableUpdater::new(&state_schedule, &mode_schedule, &self.mode_dynamics_map, self.dt);
        let mut old_value = self.evaluate_cost(&mode_schedule, &state_schedule, &self.mode_dynamics_map);

        for _ in 0..self.max_iterations {
            let d = InsertionGradientCalculator::compute(&mode_schedule, &state_schedule, &adjoint_schedule, &self.mode_dynamics_map, self.t_index0, self.t_index_last);
            let new_schedule = self.apply_armijo(&d, &mode_schedule, &state_schedule, &self.mode_dynamics_map, &self.x0, &self.propagator, &self.dt);
            let new_value = self.evaluate_cost(&new_schedule, &state_schedule, &self.mode_dynamics_map);

            if new_value - old_value <= 0.01 {
                return new_schedule;
            }
            mode_schedule = new_schedule;
            state_schedule = StateUpdater::new(&self.x0, &mode_schedule, &self.mode_dynamics_map, &self.propagator, self.dt);
            adjoint_schedule = AdjointVariableUpdater::new(&state_schedule, &mode_schedule, &self.mode_dynamics_map, self.dt);
            old_value = new_value;
        }
        mode_schedule
    }

    pub fn apply_armijo(
        &self,
        d: &Array2<f64>,
        mode_schedule: &ModeSchedule,
        state_schedule0: &StateSchedule<T>,
        mode_dynamics_map: &ModeDynamicsMap<T, U>,
        x0: &T,
        propagator: &P,
        dt: &f64,
    ) -> ModeSchedule
    {
        let d_min = d.iter().cloned().fold(f64::INFINITY, f64::min);
        let mut lambda = self.lambda;
        let old_value = self.evaluate_cost(mode_schedule, &state_schedule0, mode_dynamics_map);
        
        loop {
            let (subset_times, subset_modes) = self.get_subset(d, lambda * d_min, self.dt);

            let mut new_schedule = mode_schedule.clone();
            for (&t, &m) in subset_times.iter().zip(subset_modes.iter()) {
                new_schedule.insert(t, m);
            }
            
            let state_schedule_new = StateUpdater::new(x0, &new_schedule, mode_dynamics_map, propagator, dt.clone());

            let cost_value_new = self.evaluate_cost(&new_schedule, &state_schedule_new, mode_dynamics_map);

            if cost_value_new - old_value <= self.alpha * lambda * d_min {
                return ModeSchedule::new(0, self.t_index_last, &new_schedule.schedule);
            } else {
                lambda *= self.beta;
            }
        }
    }

    fn get_subset(&self, d: &Array2<f64>, threshold: f64, dt: f64) -> (Vec<usize>, Vec<ModeId>) {
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
            measure += dt;
            if measure > self.lambda {
                break;
            }
            subset_times.push(t);
            subset_modes.push(ModeId::new(m));
        }
        (subset_times, subset_modes)
    }

    fn evaluate_cost(
        &self,
        schedule: &ModeSchedule,
        state_schedule: &StateSchedule<T>,
        mode_dynamics_map: &ModeDynamicsMap<T, U>,
    ) -> f64
    {
        let mut cost_value = 0.0;

        for t_index in self.t_index0..self.t_index_last + 1 {
            let x = state_schedule.get(t_index).unwrap();
            let mode = schedule.get(t_index).unwrap();
            let cost = mode_dynamics_map.get_cost(*mode).unwrap();

            if t_index == self.t_index_last {
                cost_value += cost.terminal_cost(x);
            } else {
                cost_value += cost.stage_cost(x, &U::zeros());
            }
        }
        cost_value
    }
}
