use super::disturbance_trait::DisturbanceCalculator;
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::domain::state::state_trait::StateVector;
use crate::domain::{math::formulations::Math, state::position_velocity_state_eci::PositionVelocityStateEci};
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::force::force_trait::Force;
use crate::settings::constants::CONSTANTS;

use std::f64::consts::PI;
use ndarray::{Array1, arr1};
use statrs::function::erf::erf;

#[derive(Debug, Clone)]
pub struct Surface {
    pub normal_direction: Array1<f64>,
    pub area_m2: f64,
    pub air_specularity: f64,
}

pub trait AirDragForInertiaState<T: StateVector> {
    fn get_position(&self, state: &T) -> Array1<f64>;
    fn get_velocity(&self, state: &T) -> Array1<f64>;
    fn get_radius(&self) -> f64;
    fn get_air_properties(&self) -> (f64, f64, f64, f64, f64); // (mass, molecular_weight, wall_temp, molecular_temp, boltzmann_const)
    fn get_surfaces(&self) -> &Vec<Surface>;

    fn calc_air_density(&self, altitude: f64) -> f64 {
        let altitude_km = altitude / 1000.0;
        let (scale_height_km, base_height_km, base_rho_kg_m3) = match altitude_km {
            a if a > 1000.0 => (268.0, 1000.0, 3.019E-15),
            a if (900.0..1000.0).contains(&a) => (181.05, 900.0, 5.245E-15),
            a if (800.0..900.0).contains(&a) => (124.64, 800.0, 1.170E-14),
            a if (700.0..800.0).contains(&a) => (88.667, 700.0, 3.614E-14),
            a if (600.0..700.0).contains(&a) => (71.835, 600.0, 1.454E-13),
            a if (500.0..600.0).contains(&a) => (63.822, 500.0, 6.967E-13),
            a if (400.0..500.0).contains(&a) => (58.515, 400.0, 3.725E-12),
            a if (300.0..400.0).contains(&a) => (53.298, 350.0, 9.158E-12),
            a if (200.0..300.0).contains(&a) => (45.546, 250.0, 7.248E-11),
            a if (100.0..200.0).contains(&a) => (37.105, 200.0, 2.789E-10),
            a if (80.0..100.0).contains(&a) => (9.473, 120.0, 2.438E-8),
            a if (60.0..80.0).contains(&a) => (7.714, 60.0, 3.206E-4),
            a if (40.0..60.0).contains(&a) => (7.554, 40.0, 3.972E-3),
            a if (20.0..40.0).contains(&a) => (6.682, 30.0, 1.774E-2),
            a if (0.0..20.0).contains(&a) => (7.249, 0.0, 1.225),
            _ => (7.249, 0.0, 0.0),
        };

        base_rho_kg_m3 * f64::exp(-(altitude_km - base_height_km) / scale_height_km)
    }

    fn calc_function_pi(&self, s: f64) -> f64 {
        let erfs = erf(s);
        s * (-s * s).exp() + (PI.sqrt() * (s * s + 0.5) * (1.0 + erfs))
    }

    fn calc_function_chi(&self, s: f64) -> f64 {
        let erfs = erf(s);
        (-s * s).exp() + PI.sqrt() * s * (1.0 + erfs)
    }

    fn calc_force_(&self, state: &T) -> Array1<f64> {
        let altitude = self.get_position(state).dot(&self.get_position(state)).sqrt() - self.get_radius();
        let velocity_norm = self.get_velocity(state).dot(&self.get_velocity(state)).sqrt();
        let air_density = self.calc_air_density(altitude);
        let (mass, molecular_weight, wall_temp, molecular_temp, boltzmann_const) = self.get_air_properties();

        let speed = (molecular_weight * velocity_norm * velocity_norm
            / (2.0 * boltzmann_const * wall_temp))
            .sqrt();

        let mut force = arr1(&[0.0, 0.0, 0.0]);

        for surface in self.get_surfaces() {
            let cos_theta = surface.normal_direction.dot(&self.get_velocity(state)) / velocity_norm;
            if cos_theta > 0.0 {
                continue;
            }

            let sin_theta = (1.0 - cos_theta.powi(2)).sqrt();
            let speed_n = -speed * cos_theta;
            let speed_t = -speed * sin_theta;
            let diffuse = 1.0 - surface.air_specularity;

            let cn = (2.0 - diffuse) / PI.sqrt() * self.calc_function_pi(speed_n) / (speed * speed)
                + diffuse / 2.0 * self.calc_function_chi(speed_n) / (speed * speed)
                    * (wall_temp / molecular_temp).sqrt();

            let ct = diffuse * speed_t * self.calc_function_chi(speed_n) / (PI.sqrt() * speed * speed);

            let k = 0.5 * air_density * velocity_norm.powi(2) * surface.area_m2;
            let normal_coefficient = k * cn;
            let tangential_coefficient = k * ct;

            let tangential_direction = Math::normalize(&Math::cross_product(&self.get_velocity(state), &surface.normal_direction));

            force = &force + normal_coefficient * &surface.normal_direction + tangential_coefficient * tangential_direction;
        }

        force /= mass;
        force
    }
}


#[derive(Debug)]
pub struct AirDragStateEci {
    molecular_weight: f64,
    wall_temperature: f64,
    molecular_temperature: f64,
    mass: f64,
    surfaces: Vec<Surface>,
    boltzmann_constant: f64,
    radius: f64
}


impl AirDragStateEci {
    pub fn new(
        molecular_weight: f64,
        wall_temperature: f64,
        molecular_temperature: f64,
        mass: f64,
        surfaces: Vec<Surface>,
    ) -> Self {
        Self {
            molecular_weight,
            wall_temperature,
            molecular_temperature,
            mass,
            surfaces,
            boltzmann_constant: CONSTANTS.boltzmann_constant, // J/K
            radius: CONSTANTS.radius
        }
    }
}

impl AirDragForInertiaState<PositionVelocityStateEci> for AirDragStateEci {
    fn get_position(&self, state: &PositionVelocityStateEci) -> Array1<f64> {
        state.position()
    }

    fn get_velocity(&self, state: &PositionVelocityStateEci) -> Array1<f64> {
        state.velocity()
    }

    fn get_radius(&self) -> f64 {
        self.radius
    }

    fn get_air_properties(&self) -> (f64, f64, f64, f64, f64) {
        (   self.mass, 
            self.molecular_weight, 
            self.wall_temperature, 
            self.molecular_temperature, 
            self.boltzmann_constant)
    }

    fn get_surfaces(&self) -> &Vec<Surface> {
        &self.surfaces
    }
}


impl DisturbanceCalculator<PositionVelocityStateEci, Force3dEci> for AirDragStateEci {
    fn calc_force(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        Force3dEci::form_from_array(self.calc_force_(state_eci))
    }
}


#[derive(Debug)]
pub struct AirDragStatePairEci {
    molecular_weight: f64,
    wall_temperature: f64,
    molecular_temperature: f64,
    mass: f64,
    surfaces: Vec<Surface>,
    boltzmann_constant: f64,
    radius: f64 
}

impl AirDragStatePairEci {
    pub fn new(
        molecular_weight: f64,
        wall_temperature: f64,
        molecular_temperature: f64,
        mass: f64,
        surfaces: Vec<Surface>,
    ) -> Self {
        Self {
            molecular_weight,
            wall_temperature,
            molecular_temperature,
            mass,
            surfaces,
            boltzmann_constant: CONSTANTS.boltzmann_constant, // J/K
            radius: CONSTANTS.radius
        }
    }
}

impl AirDragForInertiaState<PositionVelocityPairStateEci> for AirDragStatePairEci {
    fn get_position(&self, state: &PositionVelocityPairStateEci) -> Array1<f64> {
        state.deputy().position()
    }

    fn get_velocity(&self, state: &PositionVelocityPairStateEci) -> Array1<f64> {
        state.deputy().velocity()
    }

    fn get_radius(&self) -> f64 {
        self.radius
    }

    fn get_air_properties(&self) -> (f64, f64, f64, f64, f64) {
        (   self.mass, 
            self.molecular_weight, 
            self.wall_temperature, 
            self.molecular_temperature, 
            self.boltzmann_constant)
    }

    fn get_surfaces(&self) -> &Vec<Surface> {
        &self.surfaces
    }
}


impl DisturbanceCalculator<PositionVelocityPairStateEci, Force3dEci> for AirDragStatePairEci {
    fn calc_force(&self, state_eci: &PositionVelocityPairStateEci) -> Force3dEci {
        Force3dEci::form_from_array(self.calc_force_(state_eci))
    }
}
