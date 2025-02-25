use super::disturbance_trait::DisturbanceCalculator;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::domain::state::orbital_elements::OrbitalElements;
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::force::force_3d_lvlh::Force3dLvlh;
use crate::domain::force::force_converter::ForceConverter;
use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;
use crate::domain::state::state_converter::StateConverter;
use crate::settings::constants::CONSTANTS;
use ndarray::{Array1, arr1};


pub trait J2ForInertiaState<T: StateVector> {
    fn get_position(&self, state: &T) -> Array1<f64>;
    fn get_state_eci(&self, state: &T) -> PositionVelocityStateEci;
    fn get_radius(&self) -> f64;
    fn get_mu(&self) -> f64;
    fn get_j2(&self) -> f64;

    fn calc_force_(&self, state: &T) -> Array1<f64> {
        let r = self.get_position(state).dot(&self.get_position(state)).sqrt();
        let factor = 3.0 * self.get_mu() * self.get_j2() * self.get_radius().powi(2) / (2.0 * r.powi(4));

        let oe: &OrbitalElements = &self.get_state_eci(state).convert();
        let theta = oe.nu_rad + oe.omega_rad;

        let fx = factor * (3.0 * theta.sin().powi(2) * oe.i_rad.sin().powi(2) - 1.0);
        let fy = -factor * (2.0 * theta).sin() * oe.i_rad.sin().powi(2);
        let fz = -factor * theta.sin() * (2.0 * oe.i_rad).sin();

        arr1(&[fx, fy, fz])
    }
}

#[derive(Debug)]
pub struct J2StateEci {
    j2: f64,
    mu: f64,
    radius: f64,
}

impl J2StateEci {
    pub fn new() -> Self {
        Self {
             j2: CONSTANTS.j2,
             mu: CONSTANTS.mu, 
             radius: CONSTANTS.radius }
    }
}

impl J2ForInertiaState<PositionVelocityStateEci> for J2StateEci {
    fn get_position(&self, state: &PositionVelocityStateEci) -> Array1<f64> {
        state.position()
    }

    fn get_state_eci(&self, state: &PositionVelocityStateEci) -> PositionVelocityStateEci {
        state.clone()
    }

    fn get_radius(&self) -> f64 {
        self.radius
    }

    fn get_mu(&self) -> f64 {
        self.mu
    }

    fn get_j2(&self) -> f64 {
        self.j2
    }
}

impl DisturbanceCalculator<PositionVelocityStateEci, Force3dEci> for J2StateEci {
    fn calc_force(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        let force_lvlh = Force3dLvlh::form_from_array(self.calc_force_(state_eci));
        force_lvlh.convert(&self.get_state_eci(state_eci))
    }
}

#[derive(Debug)]
pub struct J2StatePairEci {
    j2: f64,
    mu: f64,
    radius: f64,
}

impl J2StatePairEci {
    pub fn new() -> Self {
        Self {
             j2: CONSTANTS.j2,
             mu: CONSTANTS.mu, 
             radius: CONSTANTS.radius }
    }
}

impl J2ForInertiaState<PositionVelocityPairStateEci> for J2StatePairEci {
    fn get_position(&self, state: &PositionVelocityPairStateEci) -> Array1<f64> {
        let state_vec: Vec<PositionVelocityStateEci> = state.convert();
        let state_deputy = &state_vec[0];
        state_deputy.position()
    }

    fn get_state_eci(&self, state: &PositionVelocityPairStateEci) -> PositionVelocityStateEci {
        let state_vec: Vec<PositionVelocityStateEci> = state.convert();
        state_vec[0].clone() // deputy
    }

    fn get_radius(&self) -> f64 {
        self.radius
    }

    fn get_mu(&self) -> f64 {
        self.mu
    }

    fn get_j2(&self) -> f64 {
        self.j2
    }
}

impl DisturbanceCalculator<PositionVelocityPairStateEci, Force3dEci> for J2StatePairEci {
    fn calc_force(&self, state_eci: &PositionVelocityPairStateEci) -> Force3dEci {
        let force_lvlh = Force3dLvlh::form_from_array(self.calc_force_(state_eci));
        force_lvlh.convert(&self.get_state_eci(state_eci))
    }
}
