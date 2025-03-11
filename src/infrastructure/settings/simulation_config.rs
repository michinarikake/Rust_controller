#[allow(unused_imports)]
use crate::infrastructure::factory::simulator_factory::{SimulationConfig, InitializationTypeEnum, SimulationConstants, DisturbanceEnum};
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
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
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
use crate::domain::force::force_trait::Force;
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


// pair,simgle : eci
// hcw : lvlh
pub type StateType = PositionVelocityPairStateEci;
// pub type StateType = PositionVelocityStateLvlh;
// pub type StateType = PositionVelocityStateEci;

// pub type ForceType = Force3dLvlh;
// pub type ForceType = Force3dEci;
pub type ForceType = Force6dEci;

// pub type PropagatorType = EulerPropagator;
pub type PropagatorType = RungeKutta4Propagator;

pub type DynamicsType = PairTwoBodyDynamics;
// pub type DynamicsType = HcwDynamics;
// pub type DynamicsType = TwoBodyDynamics;

pub fn default_simulation_config() -> SimulationConfig {
     default_pair_simulation_config()
    //  default_single_simulation_config()
    //  default_hcw_simulation_config()
}

pub fn default_pair_simulation_config() -> SimulationConfig {
    let specularity = 0.4;
    let a0 = 2.0;
    let a1 = 0.2;

    let surface_list_chief = vec![
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[1.0, 0.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 1.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 0.0, 1.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[-1.0, 0.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, -1.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 0.0, -1.0]) },
    ];
    let surface_list_deputy = vec![
                Surface { air_specularity: specularity, area_m2: a1, normal_direction: arr1(&[1.0, 0.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a1, normal_direction: arr1(&[0.0, 1.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a1, normal_direction: arr1(&[0.0, 0.0, 1.0] )},
                Surface { air_specularity: specularity, area_m2: a1, normal_direction: arr1(&[-1.0, 0.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a1, normal_direction: arr1(&[0.0, -1.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a1, normal_direction: arr1(&[0.0, 0.0, -1.0]) },
    ];
    SimulationConfig {
        initialization: InitializationTypeEnum::OrbitalElements,
        init_data: vec![
            6928000.0,  // Semi-major axis (m)
            0.00000,      // Eccentricity
            1.57079633, // Inclination (rad)
            0.0,        // Argument of perigee (rad)
            0.28869219, // Longitude of ascending node (rad)
            0.0,        // True anomaly (rad)
            6928000.0,  // Semi-major axis (m)
            0.00000,      // Eccentricity
            1.57079633, // Inclination (rad)
            0.0,        // Argument of perigee (rad)
            0.28869219, // Longitude of ascending node (rad)
            0.0000008  // True anomaly (rad)
        ],
        disturbances: vec![
            DisturbanceEnum::AirDrag,
            // DisturbanceEnum::J2,
        ],
        constants: SimulationConstants {
            dt: 0.02,        // Time step (s)
            step: 5000,     // Time step num
            t0: 0.0,
            a: 7000000.0,    // Reference semi-major axis (m)
            molecular_weight_chief: 18.0,
            wall_temperature_chief: 30.0,
            molecular_temperature: 3.0,
            mass_chief: 50.0,
            surfaces_chief:surface_list_chief,
            molecular_weight_deputy: 18.0,
            wall_temperature_deputy: 30.0,
            mass_deputy: 50.0,
            surfaces_deputy:surface_list_deputy,
        },
    }
}

pub fn default_single_simulation_config() -> SimulationConfig {
    let specularity = 0.4;
    let a0 = 2.0;

    let surface_list = vec![
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[1.0, 0.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 1.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 0.0, 1.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[-1.0, 0.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, -1.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 0.0, -1.0]) },
    ];
    SimulationConfig {
        initialization: InitializationTypeEnum::OrbitalElements,
        init_data: vec![
            6928000.0,  // Semi-major axis (m)
            0.0011,      // Eccentricity
            1.57079633, // Inclination (rad)
            0.0,        // Argument of perigee (rad)
            0.28869219, // Longitude of ascending node (rad)
            0.0,        // True anomaly (rad)
        ],
        disturbances: vec![
            DisturbanceEnum::AirDrag,
            // DisturbanceEnum::J2,
        ],
        constants: SimulationConstants {
            dt: 1.0,        // Time step (s)
            step: 30000,     // Time step num
            t0: 0.0,
            a: 7000000.0,    // Reference semi-major axis (m)
            molecular_weight_chief: 18.0,
            wall_temperature_chief: 30.0,
            molecular_temperature: 3.0,
            mass_chief: 50.0,
            surfaces_chief:surface_list.clone(),
            molecular_weight_deputy: 18.0,
            wall_temperature_deputy: 30.0,
            mass_deputy: 50.0,
            surfaces_deputy:surface_list,
        },
    }
}

pub fn default_hcw_simulation_config() -> SimulationConfig {
    let specularity = 0.4;
    let a0 = 2.0;

    let surface_list = vec![
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[1.0, 0.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 1.0, 0.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 0.0, 1.0] )},
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[-1.0, 0.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, -1.0, 0.0]) },
                Surface { air_specularity: specularity, area_m2: a0, normal_direction: arr1(&[0.0, 0.0, -1.0]) },
    ];
    SimulationConfig {
        initialization: InitializationTypeEnum::RelativePositionVelocity,
        init_data: vec![
            10.0,
            10.0,
            10.0,
            0.0,
            0.0,
            0.0,
        ],
        disturbances: vec![
            // DisturbanceEnum::AirDrag,
            // DisturbanceEnum::J2,
        ],
        constants: SimulationConstants {
            dt: 1.0,        // Time step (s)
            step: 30000,     // Time step num
            t0: 0.0,
            a: 7000000.0,    // Reference semi-major axis (m)
            molecular_weight_chief: 18.0,
            wall_temperature_chief: 30.0,
            molecular_temperature: 3.0,
            mass_chief: 50.0,
            surfaces_chief:surface_list.clone(),
            molecular_weight_deputy: 18.0,
            wall_temperature_deputy: 30.0,
            mass_deputy: 50.0,
            surfaces_deputy:surface_list,
        },
    }
}
