use std::marker::PhantomData;
use std::collections::HashMap;


pub struct ModeTransition<T: StateVector> {
    pub transitions: HashMap<usize, usize>, // モード遷移マップ
    pub state_jumps: HashMap<(usize, usize), T>, // 状態のジャンプ
}

pub struct ModeOptimizer<T, U, D, C>
where
    T: StateVector,
    U: Force,
    D: Dynamics<T, U> + Differentiable<T, U>,
    C: Cost<T, U> + Differentiable<T, U>,
{
    dynamics: Vec<D>,
    cost: C,
    transitions: ModeTransition<T>,
    current_mode: usize,
    dt: f64,
    range_search: f64,
    x_nominal_list: Vec<T>,
    v_nominal_list: Vec<U>,
    rho_list: Vec<T>,
    v_mode_list: Vec<U>, // modeの数だけvを持つ
    _marker: PhantomData<(T, U)>,
}

impl<T, U, D, C> ModeOptimizer<T, U, D, C>
where
    T: StateVector,
    U: Force,
    D: Dynamics<T, U> + Differentiable<T, U>,
    C: Cost<T, U> + Differentiable<T, U>,
{
    pub fn compute_measure(&self, S: &Vec<f64>, t_step: f64, epsilon: f64) -> f64 {
        S.len() as f64 * t_step
    }

    // すべてのモード（v_new）を探索して最小値を求める
    pub fn compute_D_sigma_for_all_mode(&self) -> (Vec<Vec<f64>>, f64, Vec<U>) {
        let mut D_sigma_s_list = Vec::new();
        let mut D_sigma_list = Vec::new();
        let mut v_opt_list = Vec::new();

        for v_new in &self.v_mode_list {
            let (D_sigma_s, D_sigma) = self.compute_D_sigma(v_new);
            D_sigma_s_list.push(D_sigma_s);
            D_sigma_list.push(D_sigma);
        }
        let D_sigma = *D_sigma_list.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();

        // 各時刻・各モードをすべて探索して、最小値を求める
        for i in 0..self.x_nominal_list.len() {
            let mut min_val = f64::MAX;
            let mut best_v = U::zeros();
            for (j, v_new) in self.v_mode_list.iter().enumerate() {
                if D_sigma_s_list[j][i] < min_val {
                    min_val = D_sigma_s_list[j][i];
                    best_v = v_new.clone();
                }
            }
            v_opt_list.push(best_v);
        }

        (D_sigma_s_list, D_sigma, v_opt_list)
    }

    // 特定のモード（v_new）に対して、時刻に関する最小値を求める
    pub fn compute_D_sigma(&self, v_new: &U) -> (Vec<f64>, f64) {
        let mut D_sigma_s = Vec::new();
        let mut D_sigma = f64::MAX;

        for i in 0..self.x_nominal_list.len() {
            let x = &self.x_nominal_list[i];
            let v = &self.v_nominal_list[i];
            let p = &self.rho_list[i]; // Replace with proper rho storage
            let D_s = self.insertion_gradient(i as f64 * self.dt, p, x, v, v_new);
            D_sigma_s.push(D_s);
            if D_s < D_sigma {
                D_sigma = D_s;
            }
        }
        (D_sigma_s, D_sigma) // D_sigma_sは特定のモードに対する勾配の時系列
    }

    fn insertion_gradient(&self, t: f64, p: &T, x: &T, v_current: &U, v_new: &U) -> f64 {
        let new_x = self.dynamics[self.current_mode].dynamics_x(self.dt, t, x, v_new);
        let prev_x = self.dynamics[self.current_mode].dynamics_x(self.dt, t, x, v_current);
        let diff = new_x.sub_vec(&prev_x);
        let p_vec = p.get_vector();
        diff.get_vector().iter().zip(p_vec.iter()).map(|(d, p)| d * p).sum()
    }
}
