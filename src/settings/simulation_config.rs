#[allow(unused_imports)]
use crate::factory::simulator_factory::{SimulationConfig, InitializationTypeEnum, SimulationConstants, DisturbanceEnum};
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
use crate::domain::force::force_3d_lvlh::Force3dLvlh;
#[allow(unused_imports)]
use crate::domain::disturbance::air_drag_disturbance::Surface;
#[allow(unused_imports)]
use ndarray::arr1;


// pair,simgle : eci
// hcw : lvlh
#[cfg(feature = "pair")]
pub type StateType = PositionVelocityPairStateEci;
#[cfg(feature = "hcw")]
pub type StateType = PositionVelocityStateLvlh;
#[cfg(all(not(feature = "pair"), not(feature = "hcw")))]
pub type StateType = PositionVelocityStateEci;

#[cfg(feature = "pair")]
pub type ForceType = Force3dEci;
#[cfg(feature = "hcw")]
pub type ForceType = Force3dLvlh;
#[cfg(all(not(feature = "pair"), not(feature = "hcw")))]
pub type ForceType = Force3dEci;

#[cfg(feature = "euler")]
pub type PropagatorType = EulerPropagator;
#[cfg(not(feature = "euler"))]
pub type PropagatorType = RungeKutta4Propagator;

#[cfg(feature = "pair")]
pub type DynamicsType = PairTwoBodyDynamics;
#[cfg(feature = "hcw")]
pub type DynamicsType = HcwDynamics;
#[cfg(all(not(feature = "pair"), not(feature = "hcw")))]
pub type DynamicsType = TwoBodyDynamics;

#[cfg(feature = "pair")]
pub fn default_simulation_config() -> SimulationConfig {
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
            6928000.0,  // Semi-major axis (m)
            0.001,      // Eccentricity
            1.57079633, // Inclination (rad)
            0.0,        // Argument of perigee (rad)
            0.28869219, // Longitude of ascending node (rad)
            0.0         // True anomaly (rad)
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
            molecular_weight: 18.0,
            wall_temperature: 30.0,
            molecular_temperature: 3.0,
            mass: 50.0,
            surfaces:surface_list,
        },
    }
}

#[cfg(all(not(feature = "pair"), not(feature = "hcw")))]
pub fn default_simulation_config() -> SimulationConfig {
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
            mu: 3.986004 * 10.0e14, // Earth's gravitational parameter (m^3/s^2)
            dt: 1.0,        // Time step (s)
            step: 30000,     // Time step num
            t0: 0.0,
            a: 7000000.0,    // Reference semi-major axis (m)
            molecular_weight: 18.0,
            wall_temperature: 30.0,
            molecular_temperature: 3.0,
            mass: 50.0,
            surfaces:surface_list,
            boltzmann_constant: 1.380649e-23,
            radius: 6378.1e3,
            j2: 1.08263e-3
        },
    }
}
