use crate::infrastructure::factory::mode_scheduler_factory::{CreateInputDefinedDynamics, ModeSchedulerConfig};

use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::differentiable::differentiable_trait::Differentiable2d;
use crate::domain::controller::mode_controller::wrapper::InputDefinedDynamics;
use crate::infrastructure::factory::simulator_factory::SimulationConfig;
use ndarray::{Array2, arr2};
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

// pub type ControllerStateType = PositionVelocityPairStateEci;
pub type ControllerStateType = PositionVelocityStateLvlh;
// pub type ControllerStateType = PositionVelocityStateEci;

pub type ControllerForceType = Force3dLvlh;
// pub type ControllerForceType = Force3dEci;
// pub type ControllerForceType = Force6dEci;

// pub type ControllerPropagatorType = EulerPropagator;
pub type ControllerPropagatorType = RungeKutta4Propagator;

pub type ControllerDynamicsType = LinearControlledDynamics;


/// **デフォルトの `ModeSchedulerConfig`**
pub fn default_mode_scheduler_config() -> ModeSchedulerConfig {
    ModeSchedulerConfig {
        eta: 0.9,
        alpha: 0.5,
        beta: 0.5,
        max_iterations: 100,
        u_max: 0.01,
        q_matrix: Array2::<f64>::eye(6) * 0.00000000001,
        r_matrix: Array2::<f64>::zeros((3, 3)),
        qf_matrix: Array2::<f64>::eye(6),
    }
}

pub struct LinearControlledDynamics {
    a_matrix: Array2<f64>,
    b_matrix: Array2<f64>,
    control_input: Force3dLvlh,  // 固定された制御入力
}

impl CreateInputDefinedDynamics<Force3dLvlh> for LinearControlledDynamics {
    fn new(control_input: Force3dLvlh, config: &ModeSchedulerConfig, simulation_config: &SimulationConfig) -> Self {
        let a: f64 = simulation_config.constants.a;
        let n = (CONSTANTS.mu / a.powf(3.0)).powf(0.5);
        let a_matrix = arr2(&[
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [3.0 * n.powf(2.0), 0.0, 0.0, 0.0, 2.0 * n, 0.0],
            [0.0, 0.0, 0.0, -2.0 * n, 0.0, 0.0],
            [0.0, 0.0, -n.powf(2.0), 0.0, 0.0, 0.0]]);
        let b_matrix = arr2(&[
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]);
        Self { a_matrix, b_matrix, control_input }
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
