use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::dynamics::dynamics_2sat_2body::PairTwoBodyDynamics;
use crate::domain::dynamics::dynamics_2body::TwoBodyDynamics;
use crate::domain::dynamics::dynamics_hcw::HcwDynamics;
use crate::domain::dynamics::propagator::Propagator;
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::force::force_3d_lvlh::Force3dLvlh;
use crate::domain::state::orbital_elements::OrbitalElements;
use crate::factory::simulator_factory::{SimulationConfig, DisturbanceEnum, InitializationTypeEnum};
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::application::simulator::simulator::Simulator;
use crate::domain::state::state_trait::StateVector;
// use crate::settings::simulation_config::{StateType, ForceType, PropagatorType, DynamicsType};
use crate::domain::force::force_trait::Force;
use crate::domain::disturbance::air_drag_disturbance::{AirDragStateEci, AirDragStatePairEci};
use crate::domain::disturbance::j2_disturbance::{J2StateEci, J2StatePairEci};
use crate::settings::constants::CONSTANTS;

// この実装はここでいいのか...?
pub trait InitializeState {
    fn initialize(config: &SimulationConfig) -> Self;
}

impl InitializeState for PositionVelocityPairStateEci {
    fn initialize(config: &SimulationConfig) -> Self {
        let init_data = &config.init_data;
        let init_type = &config.initialization;
        let mu = CONSTANTS.mu;
        match init_type {
            InitializationTypeEnum::PositionVelocity => {
                PositionVelocityPairStateEci::form_from_list(
                    [init_data[0], init_data[1], init_data[2], init_data[3], init_data[4], init_data[5]],
                    [init_data[6], init_data[7], init_data[8], init_data[9], init_data[10], init_data[11]],
                )
            }
            InitializationTypeEnum::OrbitalElements => {
                let chief = OrbitalElements::form_from_elements(
                    init_data[0], init_data[1], init_data[2], init_data[3],
                    init_data[4], init_data[5]
                    )
                    .expect("Invalid chief orbital elements");
                let deputy = OrbitalElements::form_from_elements(
                    init_data[6], init_data[7], init_data[8], init_data[9], 
                    init_data[10], init_data[11]
                    )
                    .expect("Invalid deputy orbital elements");
                PositionVelocityPairStateEci::form_from_orbital_elements(chief, deputy, mu)
            }
            _ => panic!("Invalid pair state initialization"),
        }
    }
}

impl InitializeState for PositionVelocityStateLvlh {
    fn initialize(config: &SimulationConfig) -> Self {
        let init_data = &config.init_data;
        let init_type = &config.initialization;
        match init_type {
            InitializationTypeEnum::RelativePositionVelocity => {
                PositionVelocityStateLvlh::form_from_list(
                    [init_data[0], init_data[1], init_data[2]],
                    [init_data[3], init_data[4], init_data[5]],
                )
            }
            _ => panic!("Invalid relative state initialization"),
        }
    }
}

impl InitializeState for PositionVelocityStateEci {
    fn initialize(config: &SimulationConfig) -> Self {
        let init_data = &config.init_data;
        let init_type = &config.initialization;
        let mu = CONSTANTS.mu;
        match init_type {
            InitializationTypeEnum::PositionVelocity => {
                PositionVelocityStateEci::form_from_list(
                    [init_data[0], init_data[1], init_data[2]],
                    [init_data[3], init_data[4], init_data[5]],
                )
            }
            InitializationTypeEnum::OrbitalElements => {
                let elements = OrbitalElements::form_from_elements(
                    init_data[0], init_data[1], init_data[2],
                    init_data[3], init_data[4], init_data[5],
                )
                    .expect("Invalid orbital elements");
                PositionVelocityStateEci::form_from_orbital_elements(&elements, mu)
            }
            _ => panic!("Invalid single state initialization"),
        }
    }
}

// ここも
pub trait InitializeDynamics {
    fn initialize(config: &SimulationConfig) -> Self;
}

impl InitializeDynamics for PairTwoBodyDynamics {
    fn initialize(config: &SimulationConfig) -> Self {
        Self::new()
    }
}

impl InitializeDynamics for TwoBodyDynamics {
    fn initialize(config: &SimulationConfig) -> Self {
        Self::new()
    }
}

impl InitializeDynamics for HcwDynamics {
    fn initialize(config: &SimulationConfig) -> Self {
        Self::new(config.constants.a)
    }
}

pub trait DisturbanceInitializer<T, U> 
where
    T: StateVector + Clone,
    U: Force + Clone,
{
    fn initialize_disturbances(config: &SimulationConfig, simulator: &mut Simulator<T, U, impl Propagator<T, U>, impl ContinuousDynamics<T, U>>);
}


impl DisturbanceInitializer<PositionVelocityPairStateEci, Force3dEci> for PositionVelocityPairStateEci {
    fn initialize_disturbances(
        config: &SimulationConfig,
        simulator: &mut Simulator<PositionVelocityPairStateEci, Force3dEci, impl Propagator<PositionVelocityPairStateEci, Force3dEci>, impl ContinuousDynamics<PositionVelocityPairStateEci, Force3dEci>>,
    ) {
        for disturbance_type in config.disturbances.iter() {
            match disturbance_type {
                DisturbanceEnum::AirDrag => {
                    simulator.add_disturbance(Box::new(AirDragStatePairEci::new(
                        config.constants.molecular_weight,
                        config.constants.wall_temperature,
                        config.constants.molecular_temperature,
                        config.constants.mass,
                        config.constants.surfaces.clone(),
                    )));
                }
                DisturbanceEnum::J2 => {
                    simulator.add_disturbance(Box::new(J2StatePairEci::new()));
                }
            }
        }
    }
}

impl DisturbanceInitializer<PositionVelocityStateEci, Force3dEci> for PositionVelocityStateEci {
    fn initialize_disturbances(
        config: &SimulationConfig,
        simulator: &mut Simulator<PositionVelocityStateEci, Force3dEci, impl Propagator<PositionVelocityStateEci, Force3dEci>, impl ContinuousDynamics<PositionVelocityStateEci, Force3dEci>>,
    ) {
        for disturbance_type in config.disturbances.iter() {
            match disturbance_type {
                DisturbanceEnum::AirDrag => {
                    simulator.add_disturbance(Box::new(AirDragStateEci::new(
                        config.constants.molecular_weight,
                        config.constants.wall_temperature,
                        config.constants.molecular_temperature,
                        config.constants.mass,
                        config.constants.surfaces.clone(),
                    )));
                }
                DisturbanceEnum::J2 => {
                    simulator.add_disturbance(Box::new(J2StateEci::new()));
                }
            }
        }
    }
}

impl DisturbanceInitializer<PositionVelocityStateLvlh, Force3dLvlh> for PositionVelocityStateEci {
    fn initialize_disturbances(
        config: &SimulationConfig,
        simulator: &mut Simulator<PositionVelocityStateLvlh, Force3dLvlh, impl Propagator<PositionVelocityStateLvlh, Force3dLvlh>, impl ContinuousDynamics<PositionVelocityStateLvlh, Force3dLvlh>>,
    ){}
}
