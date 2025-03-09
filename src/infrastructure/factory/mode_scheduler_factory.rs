use std::collections::HashMap;

use crate::domain::controller::mode_controller::mode_optimizer::ModeScheduler;
use crate::domain::force::force_trait::Force;
use crate::domain::state::state_trait::StateVector;
use crate::domain::dynamics::propagator::Propagator;
// use crate::infrastructure::factory::mode_scheduler_factory::mode_dynamics_map::create_mode_dynamics_map;
use crate::infrastructure::factory::simulator_factory::SimulationConfig;
use crate::domain::controller::mode_controller::mode_optimizer::{ModeId, ModeDynamicsMap, ContinuousDynamicsAndDifferentiable, CostAndDifferentiable};
use crate::domain::cost::quadric_cost::QuadraticCost;
use crate::infrastructure::factory::initialization_wrapper::InitializeState;
use crate::domain::state::state_converter::StateConverter;
use crate::infrastructure::settings::mode_shcedule_settings::{CreateInputDefinedDynamics, ModeSchedulerConfig};

// use crate::infrastructure::settings::simulation_config;

use ndarray::arr1;

pub struct ControllerFactory<T, T2, U, P, D> {
    _phantom: std::marker::PhantomData<(T, T2, U, P, D)>,
}


impl<T, T2, U, P, D> ControllerFactory<T, T2, U, P, D>
where 
    T: StateVector + Clone + 'static,
    T2: StateVector + Clone + 'static + StateConverter<T>,
    U: Force + Clone + 'static,
    P: Propagator<T, U> + 'static,
    D: ContinuousDynamicsAndDifferentiable<T, U> + CreateInputDefinedDynamics<U> + 'static,
{
    /// **27通りの制御入力を考慮した `ModeDynamicsMap` を作成**
    pub fn create_mode_dynamics_map(simulation_config: &SimulationConfig, config: &ModeSchedulerConfig) -> ModeDynamicsMap<T, U> {
        let mut dynamics_mapping = HashMap::new();
        let mut cost_mapping = HashMap::new();

        let q_matrix = config.q_matrix.clone();
        let r_matrix = config.r_matrix.clone();
        let qf_matrix = config.qf_matrix.clone();
        
        // 制御入力の組み合わせ (-1,0,1) の 27 通り
        let u_max = config.u_max;
        for (i, &ux) in [0.0, u_max, -u_max].iter().enumerate() {
            for (j, &uy) in [0.0, u_max, -u_max].iter().enumerate() {
                for (k, &uz) in [0.0, u_max, -u_max].iter().enumerate() {
                    let mode_id = ModeId::new(i * 9 + j * 3 + k);
                    let control_input = U::form_from_array(arr1(&[ux, uy, uz]));
                    let dynamics = D::new(control_input.clone(), &config, &simulation_config);
                    let cost = QuadraticCost::new(q_matrix.clone(), r_matrix.clone(), qf_matrix.clone(), 100.0);

                    dynamics_mapping.insert(mode_id, Box::new(dynamics) as Box<dyn ContinuousDynamicsAndDifferentiable<T, U>>);
                    cost_mapping.insert(mode_id, Box::new(cost) as Box<dyn CostAndDifferentiable<T, U>>);
                }
            }
        }

        ModeDynamicsMap::new(dynamics_mapping, cost_mapping)
    }


    pub fn create_mode_scheduler(x0_simulator: &T2, simulation_config: &SimulationConfig, config: &ModeSchedulerConfig) -> ModeScheduler<T, U, P> {
        let mode_dynamics_map = Self::create_mode_dynamics_map(simulation_config, config);
        let propagator = P::new(std::marker::PhantomData::<(T, U)>);
        let x0: T = x0_simulator.convert();

        ModeScheduler::new(
            config.eta,
            config.alpha,
            config.beta,
            config.max_iterations,
            0,
            simulation_config.constants.step as usize,
            x0,
            mode_dynamics_map.dynamics_mapping,
            mode_dynamics_map.cost_mapping,
            propagator,
            simulation_config.constants.dt,
        )
    }
}