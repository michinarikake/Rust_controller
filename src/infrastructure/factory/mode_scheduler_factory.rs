pub mod linear_controllerd_dynamics;
pub mod mode_dynamics_map;

use crate::domain::controller::mode_controller::mode_optimizer::ModeScheduler;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::state::state_trait::StateVector;
use crate::domain::dynamics::propagator::RungeKutta4Propagator;
use crate::infrastructure::factory::mode_scheduler_factory::mode_dynamics_map::create_mode_dynamics_map;


use ndarray::arr1;

/// **設定値**
pub struct ModeSchedulerConfig {
    pub eta: f64,
    pub alpha: f64,
    pub beta: f64,
    pub lambda: f64,
    pub max_iterations: usize,
    pub t_index0: usize,
    pub t_index_last: usize,
    pub dt: f64,
}

/// **デフォルトの `ModeSchedulerConfig`**
pub fn default_mode_scheduler_config() -> ModeSchedulerConfig {
    ModeSchedulerConfig {
        eta: 0.01,
        alpha: 0.1,
        beta: 0.9,
        lambda: 0.1,
        max_iterations: 100,
        t_index0: 0,
        t_index_last: 100,
        dt: 1.0,
    }
}

/// **`ModeScheduler` の作成**
pub fn create_mode_scheduler(config: &ModeSchedulerConfig) -> ModeScheduler<PositionVelocityStateEci, Force3dEci, RungeKutta4Propagator> {
    let mode_dynamics_map = create_mode_dynamics_map();
    let propagator = RungeKutta4Propagator;
    let x0 = PositionVelocityStateEci::form_from_array(arr1(&[0.0; 6]));

    ModeScheduler::new(
        config.eta,
        config.alpha,
        config.beta,
        config.lambda,
        config.max_iterations,
        config.t_index0,
        config.t_index_last,
        x0,
        mode_dynamics_map.dynamics_mapping,
        mode_dynamics_map.cost_mapping,
        propagator,
        config.dt,
    )
}
