use crate::domain::dynamics::propagator::Propagator;
use crate::domain::state::orbital_elements::OrbitalElements;
// use crate::domain::state::position_velocity_covariance_state_lvlh::PositionVelocityCovarianceStateLvlh;
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::application::simulator::simulator::Simulator;
use crate::settings::simulation_config::{StateType, ForceType, PropagatorType, DynamicsType};

#[derive(Debug)]
pub enum InitializationTypeEnum {
    PositionVelocity,
    OrbitalElements,
    RelativePositionVelocity,
}

#[derive(Debug)]
pub struct SimulationConfig{
    pub initialization: InitializationTypeEnum,
    pub init_data: Vec<f64>,
    pub constants: SimulationConstants
}

#[derive(Debug)]
pub struct SimulationConstants {
    pub mu: f64, // Earth's gravitational parameter (m^3/s^2)
    pub dt: f64,        // Time step (s)
    pub step: i64,        // Time step (s)
    pub t0: f64,         // Time start (s)
    pub a: f64,    // Reference semi-major axis (m)
}

pub struct SimulatorFactory;

impl SimulatorFactory {
    pub fn create_simulator(
        config: &SimulationConfig,
    ) -> Simulator<StateType, ForceType, PropagatorType, DynamicsType> {
        // `type alias` は `struct` なので、インスタンス化が必要
        let propagator = PropagatorType::new(std::marker::PhantomData::<(StateType, ForceType)>);
        let dynamics = DynamicsType::new(config.constants.mu);

        // `DynamicsType` に基づいて適切な初期化関数を呼ぶ
        #[cfg(feature = "pair")]
        let state = Self::initialize_pair_state(&config.initialization, &config.init_data, config.constants.mu);
        
        #[cfg(feature = "hcw")]
        let state = Self::initialize_relative_state(&config.initialization, &config.init_data);

        #[cfg(all(not(feature = "pair"), not(feature = "hcw")))]
        let state = Self::initialize_state(&config.initialization, &config.init_data, config.constants.mu);

        Simulator::new(propagator, dynamics, state, config.constants.dt, config.constants.step, config.constants.t0)
    }


    #[allow(unused)]
    fn initialize_state(init_type: &InitializationTypeEnum, init_data: &[f64], mu: f64) -> PositionVelocityStateEci {
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

    #[allow(unused)]
    fn initialize_pair_state(init_type: &InitializationTypeEnum, init_data: &[f64], mu: f64) -> PositionVelocityPairStateEci {
        match init_type {
            InitializationTypeEnum::PositionVelocity => {
                PositionVelocityPairStateEci::form_from_list(
                    [init_data[0], init_data[1], init_data[2], init_data[3], init_data[4], init_data[5]],
                    [init_data[6], init_data[7], init_data[8], init_data[9], init_data[10], init_data[11]],
                )
            }
            InitializationTypeEnum::OrbitalElements => {
                let chief = OrbitalElements::form_from_elements(init_data[0], init_data[1], init_data[2], init_data[3], init_data[4], init_data[5])
                    .expect("Invalid chief orbital elements");
                let deputy = OrbitalElements::form_from_elements(init_data[6], init_data[7], init_data[8], init_data[9], init_data[10], init_data[11])
                    .expect("Invalid deputy orbital elements");
                PositionVelocityPairStateEci::form_from_orbital_elements(chief, deputy, mu)
            }
            _ => panic!("Invalid pair state initialization"),
        }
    }

    #[allow(unused)]
    fn initialize_relative_state(init_type: &InitializationTypeEnum, init_data: &[f64]) -> PositionVelocityStateLvlh {
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
