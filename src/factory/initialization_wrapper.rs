use crate::models::dynamics::dynamics_trait::ContinuousDynamics;
use crate::models::dynamics::dynamics_2sat_2body::PairTwoBodyDynamics;
use crate::models::dynamics::dynamics_2body::TwoBodyDynamics;
use crate::models::dynamics::dynamics_hcw::HcwDynamics;
use crate::models::dynamics::propagator::Propagator;
use crate::models::force::force_3d_eci::Force3dEci;
use crate::models::force::force_6d_eci::Force6dEci;
use crate::models::force::force_3d_lvlh::Force3dLvlh;
use crate::models::state::orbital_elements::OrbitalElements;
use crate::factory::simulator_factory::{SimulationConfig, DisturbanceEnum, InitializationTypeEnum};
use crate::models::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::models::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::models::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::application::simulator::simulator::Simulator;
use crate::models::state::state_trait::StateVector;
use crate::models::force::force_trait::Force;
use crate::models::disturbance::air_drag_disturbance::{AirDragStateEci, AirDragStatePairEci};
use crate::models::disturbance::j2_disturbance::{J2StateEci, J2StatePairEci};
use crate::models::state::state_converter::StateConverter;

// この実装はここでいいのか...?
pub trait InitializeState {
    fn initialize(config: &SimulationConfig) -> Self;
}

impl InitializeState for PositionVelocityPairStateEci {
    fn initialize(config: &SimulationConfig) -> Self {
        let init_data = &config.init_data;
        let init_type = &config.initialization;
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
                vec![chief, deputy].convert()
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
                elements.convert()
            }
            _ => panic!("Invalid single state initialization"),
        }
    }
}

// ここも
pub trait InitializeDynamics {
    fn initialize(config: &SimulationConfig) -> Self;
}

#[allow(unused)]
impl InitializeDynamics for PairTwoBodyDynamics {
    fn initialize(config: &SimulationConfig) -> Self {
        Self::new()
    }
}

#[allow(unused)]
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


impl DisturbanceInitializer<PositionVelocityPairStateEci, Force6dEci> for PositionVelocityPairStateEci {
    fn initialize_disturbances(
        config: &SimulationConfig,
        simulator: &mut Simulator<PositionVelocityPairStateEci, Force6dEci, impl Propagator<PositionVelocityPairStateEci, Force6dEci>, impl ContinuousDynamics<PositionVelocityPairStateEci, Force6dEci>>,
    ) {
        for disturbance_type in config.disturbances.iter() {
            match disturbance_type {
                DisturbanceEnum::AirDrag => {
                    simulator.add_disturbance(Box::new(AirDragStatePairEci::new(
                        config.constants.molecular_weight_chief,
                        config.constants.wall_temperature_chief,
                        config.constants.molecular_temperature,
                        config.constants.mass_chief,
                        config.constants.surfaces_chief.clone(),
                        config.constants.molecular_weight_deputy,
                        config.constants.wall_temperature_deputy,
                        config.constants.mass_deputy,
                        config.constants.surfaces_deputy.clone(),
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
                        config.constants.molecular_weight_chief,
                        config.constants.wall_temperature_chief,
                        config.constants.molecular_temperature,
                        config.constants.mass_chief,
                        config.constants.surfaces_chief.clone(),
                    )));
                }
                DisturbanceEnum::J2 => {
                    simulator.add_disturbance(Box::new(J2StateEci::new()));
                }
            }
        }
    }
}

#[allow(unused)]
impl DisturbanceInitializer<PositionVelocityStateLvlh, Force3dLvlh> for PositionVelocityStateLvlh {
    fn initialize_disturbances(
        config: &SimulationConfig,
        simulator: &mut Simulator<PositionVelocityStateLvlh, Force3dLvlh, impl Propagator<PositionVelocityStateLvlh, Force3dLvlh>, impl ContinuousDynamics<PositionVelocityStateLvlh, Force3dLvlh>>,
    ){}
}
