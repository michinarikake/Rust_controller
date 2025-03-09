use crate::domain::state::state_converter::StateConverter;
use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::differentiable::differentiable_trait::Differentiable2d;
use crate::domain::controller::mode_controller::wrapper::InputDefinedDynamics;
use crate::infrastructure::factory::simulator_factory::SimulationConfig;
use crate::infrastructure::settings::simulation_config::default_pair_simulation_config;
use ndarray::{Array2, arr2};
use ndarray_linalg::Inverse;
use serde::de;
use crate::infrastructure::settings::constants::CONSTANTS;
#[allow(unused_imports)]
use crate::domain::dynamics::propagator::RungeKutta4Propagator;
#[allow(unused_imports)]
use crate::domain::dynamics::propagator::EulerPropagator;
#[allow(unused_imports)]
use crate::domain::dynamics::dynamics_2sat_2body::PairTwoBodyDynamics;
#[allow(unused_imports)]
use crate::domain::dynamics::dynamics_2body::TwoBodyDynamics;
#[allow(unused_imports)]
use crate::domain::dynamics::dynamics_hcw::HcwDynamics;
#[allow(unused_imports)]
use crate::domain::state::orbital_elements::OrbitalElements;
#[allow(unused_imports)]
use crate::domain::state::position_velocity_covariance_state_lvlh::PositionVelocityCovarianceStateLvlh;
#[allow(unused_imports)]
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
#[allow(unused_imports)]
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
#[allow(unused_imports)]
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
#[allow(unused_imports)]
use crate::domain::force::force_3d_eci::Force3dEci;
#[allow(unused_imports)]
use crate::domain::force::force_6d_eci::Force6dEci;
#[allow(unused_imports)]
use crate::domain::force::force_3d_lvlh::Force3dLvlh;
#[allow(unused_imports)]
use crate::domain::disturbance::air_drag_disturbance::Surface;
#[allow(unused_imports)]
use ndarray::arr1;

/// **制御入力が固定されたダイナミクス**
pub trait CreateInputDefinedDynamics<U: Force>
{
    fn new(control_input: U, config: &ModeSchedulerConfig, simulation_config: &SimulationConfig) -> Self;
}


/// **設定値**
pub struct ModeSchedulerConfig {
    pub eta: f64,
    pub alpha: f64,
    pub beta: f64,
    pub max_iterations: usize,
    pub u_max: f64,
    pub q_matrix: Array2<f64>,
    pub r_matrix: Array2<f64>,
    pub qf_matrix: Array2<f64>,
    pub a_matrix: Array2<f64>,
    pub b_matrix: Array2<f64>,
    pub c_matrix: Array2<f64>,
    pub d_matrix: Array2<f64>,
    pub f1_matrix: Array2<f64>,
    pub f2_matrix: Array2<f64>,
}

// pub type ControllerStateType = PositionVelocityPairStateEci;
// pub type ControllerStateType = PositionVelocityStateLvlh;
// pub type ControllerStateType = PositionVelocityStateEci;
pub type ControllerStateType = PositionVelocityCovarianceStateLvlh;

pub type ControllerForceType = Force3dLvlh;
// pub type ControllerForceType = Force3dEci;
// pub type ControllerForceType = Force6dEci;

// pub type ControllerPropagatorType = EulerPropagator;
pub type ControllerPropagatorType = RungeKutta4Propagator;

// pub type ControllerDynamicsType = LinearControlledDynamics;
pub type ControllerDynamicsType = PositionVelocityCovarianceDynamics;


/// **デフォルトの `ModeSchedulerConfig`**
pub fn default_mode_scheduler_config(simulation_config: &SimulationConfig) -> ModeSchedulerConfig {
    let a: f64 = simulation_config.constants.a;
    let n = (CONSTANTS.mu / a.powf(3.0)).powf(0.5);
    ModeSchedulerConfig {
        eta: 0.9,
        alpha: 0.5,
        beta: 0.5,
        max_iterations: 100,
        u_max: 0.01,
        q_matrix: Array2::<f64>::eye(33) * 0.00000000001,
        r_matrix: Array2::<f64>::zeros((3, 3)),
        qf_matrix: Array2::<f64>::eye(33),
        a_matrix: arr2(&[
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [3.0 * n.powf(2.0), 0.0, 0.0, 0.0, 2.0 * n, 0.0],
            [0.0, 0.0, 0.0, -2.0 * n, 0.0, 0.0],
            [0.0, 0.0, -n.powf(2.0), 0.0, 0.0, 0.0]]),
        b_matrix: arr2(&[
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]),
        c_matrix: arr2(&[
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        ]),
        d_matrix: arr2(&[
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]) * 0.0001,
        f1_matrix: arr2(&[
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]) * 0.0,
        f2_matrix: arr2(&[
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ]) * 0.01,
    }
}

impl StateConverter<PositionVelocityCovarianceStateLvlh> for PositionVelocityPairStateEci {
    fn convert(&self) -> PositionVelocityCovarianceStateLvlh {
        let pos_vec_lvlh: PositionVelocityStateLvlh = self.convert();
        // 初期の不確かさはとりあえず0
        let p = Array2::<f64>::zeros((6, 6));
        PositionVelocityCovarianceStateLvlh::from_from_states(&pos_vec_lvlh, &pos_vec_lvlh, p)
    }
}


// PositionVelocityCovarianceStateLvlh に対する制御入力定義ダイナミクス
pub struct PositionVelocityCovarianceDynamics {
    a_mat: Array2<f64>,
    b_mat: Array2<f64>,
    c_mat: Array2<f64>,
    d_mat: Array2<f64>,
    f1_mat: Array2<f64>,
    f2_mat: Array2<f64>,
    control_input: Force3dLvlh,  // 固定された制御入力
    epsilon: f64,
}

// impl PositionVelocityCovarianceDynamics {
//     fn delta(&self, j: usize, k: usize) -> f64 {
//         if j == k { 1.0 } else { 0.0 }
//     }

//     fn index(&self, i: usize, j: usize) -> usize {
//         assert!(i <= j, "i should be <= j for upper triangular matrix indexing");
//         i * (11 - i) / 2 + (j - i)
//     }
    
// }

#[allow(unused)]
impl CreateInputDefinedDynamics<Force3dLvlh> for PositionVelocityCovarianceDynamics {
    fn new(control_input: Force3dLvlh, config: &ModeSchedulerConfig, simulation_config: &SimulationConfig) -> Self {
        Self 
        { 
            a_mat: config.a_matrix.clone(), 
            b_mat: config.b_matrix.clone(),
            c_mat: config.c_matrix.clone(),
            d_mat: config.d_matrix.clone(),
            f1_mat: config.f1_matrix.clone(),
            f2_mat: config.f2_matrix.clone(),
            control_input,
            epsilon: 0.0000001,
        }
    }
}

impl ContinuousDynamics<PositionVelocityCovarianceStateLvlh, Force3dLvlh> for PositionVelocityCovarianceDynamics {
    fn compute_derivative(&self, state: &PositionVelocityCovarianceStateLvlh, _: &Force3dLvlh) -> PositionVelocityCovarianceStateLvlh {
        // 状態量の取得
        let mu_x = state.get_mu_x();
        let est_x = state.get_est_x();
        let p = state.get_covariance_matrix();
        let mu_x_vec = mu_x.get_vector().clone();
        let est_x_vec = est_x.get_vector().clone();
        let input_vec = self.control_input.get_vector().clone();

        // ゲイン行列Kの計算
        let k = p.dot(&self.c_mat.t()).dot(&self.d_mat.dot(&self.d_mat.t()).inv().unwrap());

        // 外部ノイズの計算
        let noise = self.f1_mat.dot(&p).dot(&self.f1_mat.t())
            + (self.f1_mat.dot(&est_x_vec) + self.f2_mat.dot(&input_vec))
                .dot(&(self.f1_mat.dot(&est_x_vec) + self.f2_mat.dot(&input_vec)).t());

        // 各導関数の計算
        let mu_x_derivative = self.a_mat.dot(&mu_x_vec) + self.b_mat.dot(&input_vec);
        let est_x_derivative = self.a_mat.dot(&est_x_vec) + self.b_mat.dot(&input_vec)
            + k.dot(&self.c_mat.dot(&(mu_x_vec - &est_x_vec)));
        // println!("{}", k.dot(&self.c_mat));
        let p_derivative = self.a_mat.dot(&p) + p.dot(&self.a_mat.t()) - k.dot(&self.c_mat.dot(&p)) + noise;


        // 新しい状態を生成
        PositionVelocityCovarianceStateLvlh::form_from_list(
            mu_x_derivative.to_vec().try_into().unwrap(),
            est_x_derivative.to_vec().try_into().unwrap(),
            p_derivative
        )
    }
    
}

impl Differentiable2d<PositionVelocityCovarianceStateLvlh, Force3dLvlh> for PositionVelocityCovarianceDynamics
{
    fn differentiate(&self, state: &PositionVelocityCovarianceStateLvlh, _: &Force3dLvlh, _: f64) -> Array2<f64> {
        let mut numerical_jacobian = Array2::<f64>::zeros((33, 33));
        let f_original = self.compute_derivative(state, &Force3dLvlh::zeros()); // f(z) の値を取得

        for j in 0..33 {
            let mut perturbed_z = state.get_vector().clone();
            perturbed_z[j] += self.epsilon; // 状態を少しずらす
            let new_state = PositionVelocityCovarianceStateLvlh::form_from_array(perturbed_z); // z + ε に対応する状態量を生成

            let f_perturbed = self.compute_derivative(&new_state, &Force3dLvlh::zeros()); // f(z + ε) を計算
            let df = (&f_perturbed - &f_original) / self.epsilon; // 数値微分

            numerical_jacobian.column_mut(j).assign(&df.get_vector()); // 列に格納
        }

        numerical_jacobian
    }
    // fn differentiate(&self, state: &PositionVelocityCovarianceStateLvlh, _: &Force3dLvlh, _: f64) -> Array2<f64> {
    //     // mu_x_dot = f1
    //     // est_x_dot = f2
    //     // vec<P_dot> = f3
    //     // jacobian =
    //     // [
    //     //     d f1 / d mu_x1, ..., d f1 / d mu_x6, d f1 / d est_x1, ..., d f1 / d est_x6, d f1 / d P11, d f1 / d P12, ..., d f1 / d P66
    //     //     d f2 / d mu_x1, ..., d f2 / d mu_x6, d f2 / d est_x1, ..., d f2 / d est_x6, d f2 / d P11, d f2 / d P12, ..., d f2 / d P66
    //     //     d f3 / d mu_x1, ..., d f3 / d mu_x6, d f3 / d est_x1, ..., d f3 / d est_x6, d f3 / d P11, d f3 / d P12, ..., d f3 / d P66
    //     // ]
    //     let mut jacobian = Array2::<f64>::zeros((33, 33));
    //     let m_mat = (self.c_mat.t().dot(&self.d_mat.dot(&self.d_mat.t()).inv().unwrap())).dot(&self.c_mat);
    //     let p_mat = state.get_covariance_matrix();
    //     let mu_x = state.get_mu_x().get_vector().clone();
    //     let est_x = state.get_est_x().get_vector().clone();
    //     let u = self.control_input.get_vector().clone();

    //     // mu_x　での微分 ==================================================
    //     // `mu_x` のヤコビアン部分 (0~6 × 0~6)
    //     for i in 0..6 {
    //         for j in 0..6 {
    //             jacobian[[i, j]] = self.a_mat[[i, j]];
    //         }
    //     }

    //     // `est_x` のmu_x依存部分 (6~12 × 0~6)
    //     for i in 0..6 {
    //         for j in 0..6 {
    //             jacobian[[i + 6, j]] = p_mat.dot(&m_mat)[[i, j]];
    //         }
    //     }

    //     // `P` のmu_x依存部分 (12~ × 0~6)はなし

    //     // est_x での微分 ==================================================
    //     // mu_x のest_x依存部分 (0~6 × 6~12)はなし
        
    //     // `est_x` のヤコビアン部分 (6~12 × 6~12)
    //     for i in 0..6 {
    //         for j in 0..6 {
    //             jacobian[[i + 6, j + 6]] = self.a_mat[[i, j]] - p_mat.dot(&m_mat)[[i, j]];
    //         }
    //     }

    //     // P のダイナミクスのest_x依存部分 (12~ × 6~12)
    //     for i in 0..6 {
    //         for k in 0..6 {
    //             for l in k..6 {
    //                 let mut derivative = 0.0;
    //                 derivative += self.f1_mat[[k, i]] * self.f1_mat.dot(&est_x)[l];
    //                 derivative += self.f1_mat[[i, l]] * self.f1_mat.dot(&est_x)[k];
    //                 derivative += self.f1_mat[[k, i]] * self.f2_mat.dot(&u)[l];
    //                 derivative += self.f1_mat[[i, l]] * self.f2_mat.dot(&u)[k];

    //                 jacobian[[self.index(k, l) + 12, i + 6]] = derivative;
    //             }
    //         }
    //     }

    //     // P での微分 ==================================================
    //     // mu_x の P 依存部分 (0~6 × 12~)はなし

    //     // est_x の P 依存部分 (6~12 × 12~)
    //     let v = m_mat.dot(&(mu_x - est_x));
    //     for i in 0..6 {
    //         for j in i..6 {
    //             for l in 0..6 {
    //                 jacobian[[l + 6, self.index(i, j) + 12]] = v[j] * self.delta(i, l);
    //             }
    //         }
    //     }

    //     // `P` のヤコビアン部分 (12~ × 12~)
    //     for i in 0..6 {
    //         for j in i..6 {
    //             for k in 0..6 {
    //                 for l in k..6 {
    //                     let mut derivative = 0.0;

    //                     // AP + PA^T の微分
    //                     derivative += self.a_mat[[k, i]] * self.delta(j, l) + self.a_mat[[l, j]] * self.delta(i, k);
    //                     derivative += self.a_mat[[k, j]] * self.delta(i, l) + self.a_mat[[l, i]] * self.delta(j, k);

    //                     // F1 P F1^T の微分
    //                     derivative += self.f1_mat[[k, i]] * self.f1_mat[[j, l]];
    //                     // derivative += self.f1_mat[[k, j]] * self.f1_mat[[i, l]];

    //                     // P C (D D^T)^-1 C P の微分
    //                     derivative -= m_mat.row(j)
    //                         .iter()
    //                         .zip(p_mat.column(l).iter())
    //                         .map(|(&m_jn, &p_nl)| m_jn * p_nl)
    //                         .sum::<f64>()
    //                         * self.delta(i, k);
    //                     // derivative -= m_mat.row(i)
    //                     //     .iter()
    //                     //     .zip(p_mat.column(k).iter())
    //                     //     .map(|(&m_in, &p_nk)| m_in * p_nk)
    //                     //     .sum::<f64>()
    //                     //     * self.delta(i, l);
    //                     derivative -= p_mat.row(k)
    //                         .iter()
    //                         .zip(m_mat.column(i).iter())
    //                         .map(|(&p_km, &m_mi)| p_km * m_mi)
    //                         .sum::<f64>()
    //                         * self.delta(j, l);
    //                     // derivative -= p_mat.row(l)
    //                     //     .iter()
    //                     //     .zip(m_mat.column(j).iter())
    //                     //     .map(|(&p_lm, &m_mj)| p_lm * m_mj)
    //                     //     .sum::<f64>()
    //                     //     * self.delta(j, k);

    //                     jacobian[[self.index(k, l) + 12, self.index(i, j) + 12]] = derivative;
    //                 }
    //             }
    //         }
    //     }

    //     jacobian
    // }
}


impl InputDefinedDynamics<PositionVelocityCovarianceStateLvlh, Force3dLvlh> for PositionVelocityCovarianceDynamics
{
    fn get_input(&self, _: &PositionVelocityCovarianceStateLvlh, _: f64) -> Force3dLvlh {
        self.control_input.clone()
    }
}


pub struct LinearControlledDynamics {
    a_matrix: Array2<f64>,
    b_matrix: Array2<f64>,
    control_input: Force3dLvlh,  // 固定された制御入力
}

#[allow(unused)]
impl CreateInputDefinedDynamics<Force3dLvlh> for LinearControlledDynamics {
    fn new(control_input: Force3dLvlh, config: &ModeSchedulerConfig, simulation_config: &SimulationConfig) -> Self {
        Self { a_matrix: config.a_matrix.clone(), b_matrix: config.b_matrix.clone(), control_input }
    }
}

impl ContinuousDynamics<PositionVelocityStateLvlh, Force3dLvlh> for LinearControlledDynamics
{
    fn compute_derivative(&self, state: &PositionVelocityStateLvlh, _: &Force3dLvlh) -> PositionVelocityStateLvlh {
        let dx = self.a_matrix.clone().dot(state.get_vector()) + (self.b_matrix.clone().dot(self.control_input.get_vector()));
        PositionVelocityStateLvlh::form_from_array(dx)
    }
}

impl Differentiable2d<PositionVelocityStateLvlh, Force3dLvlh> for LinearControlledDynamics
{
    fn differentiate(&self, _: &PositionVelocityStateLvlh, _: &Force3dLvlh, _: f64) -> Array2<f64> {
        self.a_matrix.clone()
    }
}

impl InputDefinedDynamics<PositionVelocityStateLvlh, Force3dLvlh> for LinearControlledDynamics
{
    fn get_input(&self, _: &PositionVelocityStateLvlh, _: f64) -> Force3dLvlh {
        self.control_input.clone()
    }
}

// #[cfg(test)]
// const EPSILON: f64 = 1e-5;

// #[cfg(test)]
// fn numerical_jacobian(
//     dynamics: &PositionVelocityCovarianceDynamics,
//     state: &PositionVelocityCovarianceStateLvlh,
// ) -> Array2<f64> {
//     let mut numerical_jacobian = Array2::<f64>::zeros((33, 33));
//     let f_original = dynamics.compute_derivative(state, &Force3dLvlh::zeros()); // f(z) の値を取得

//     for j in 0..33 {
//         let mut perturbed_z = state.get_vector().clone();
//         perturbed_z[j] += EPSILON; // 状態を少しずらす
//         let new_state = PositionVelocityCovarianceStateLvlh::form_from_array(perturbed_z); // z + ε に対応する状態量を生成

//         let f_perturbed = dynamics.compute_derivative(&new_state, &Force3dLvlh::zeros()); // f(z + ε) を計算
//         let df = (&f_perturbed - &f_original) / EPSILON; // 数値微分

//         numerical_jacobian.column_mut(j).assign(&df.get_vector()); // 列に格納
//     }

//     numerical_jacobian
// }

// #[test]
// fn test_jacobian_accuracy() {
//     let simulation_config = default_pair_simulation_config();
//     let config = default_mode_scheduler_config(&simulation_config);
//     let mu_x0 = arr1(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
//     let est_x0 = arr1(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
//     let p0 = arr2(&[
//         [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
//         [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
//         [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
//         [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
//         [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
//         [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
//     ]);
//     let state = PositionVelocityCovarianceStateLvlh::from_from_states(
//         &PositionVelocityStateLvlh::form_from_array(mu_x0),
//         &PositionVelocityStateLvlh::form_from_array(est_x0),
//         p0,
//     );
//     let dynamics = PositionVelocityCovarianceDynamics::new(Force3dLvlh::zeros(), &config, &simulation_config);

//     let analytical_jacobian = dynamics.differentiate(&state, &Force3dLvlh::zeros(), 0.0);
//     let numerical_jacobian = numerical_jacobian(&dynamics, &state);
//     // すべての要素を省略せずに表示
//     // for i in 0..33 {
//     //     for j in 0..33 {
//     //         println!("({}, {}), numerical: {}, analitical: {}", i, j, numerical_jacobian[[i, j]], analytical_jacobian[[i, j]]);
//     //     }
//     // }

//     // 誤差が1e-3以上のものを表示
//     for i in 0..33 {
//         for j in 0..33 {
//             if (numerical_jacobian[[i, j]] - analytical_jacobian[[i, j]]).abs() > 1e-3 {
//                 println!("({}, {}), numerical: {}, analitical: {}", i, j, numerical_jacobian[[i, j]], analytical_jacobian[[i, j]]);
//             }
//         }
//     }

//     let error = &analytical_jacobian - &numerical_jacobian;
//     let max_error = error.iter().map(|x| x.abs()).fold(0.0, f64::max);

//     println!("Max error: {:.6e}", max_error);
//     assert!(max_error < 1e-3, "Jacobian computation may be incorrect!");
// }
