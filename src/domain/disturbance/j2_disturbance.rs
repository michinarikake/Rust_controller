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
use crate::infrastructure::settings::constants::CONSTANTS;
use crate::domain::force::force_6d_eci::Force6dEci;
use ndarray::{Array1, arr1};

pub trait J2ForInertiaState<T: StateVector> {
    fn calc_force_(
        &self,
        position: Array1<f64>,
        state_eci: &PositionVelocityStateEci,
    ) -> Array1<f64> {
        let r = position.dot(&position).sqrt();
        let factor = 3.0 * CONSTANTS.mu * CONSTANTS.j2 * CONSTANTS.radius.powi(2) / (2.0 * r.powi(4));

        let oe: &OrbitalElements = &state_eci.convert();
        let theta = oe.nu_rad + oe.omega_rad;

        let fx = factor * (3.0 * theta.sin().powi(2) * oe.i_rad.sin().powi(2) - 1.0);
        let fy = -factor * (2.0 * theta).sin() * oe.i_rad.sin().powi(2);
        let fz = -factor * theta.sin() * (2.0 * oe.i_rad).sin();

        arr1(&[fx, fy, fz])
    }
}

#[derive(Debug)]
pub struct J2StateEci;

impl J2StateEci {
    pub fn new() -> Self {
        Self {}
    }
}

impl J2ForInertiaState<PositionVelocityStateEci> for J2StateEci {}

impl DisturbanceCalculator<PositionVelocityStateEci, Force3dEci> for J2StateEci {
    fn calc_force(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        let force_lvlh = Force3dLvlh::form_from_array(self.calc_force_(
            state_eci.position(),
            state_eci,
        ));
        force_lvlh.convert(state_eci)
    }
}

#[derive(Debug)]
pub struct J2StatePairEci;

impl J2StatePairEci {
    pub fn new() -> Self {
        Self {}
    }
}

impl J2ForInertiaState<PositionVelocityPairStateEci> for J2StatePairEci {}

impl DisturbanceCalculator<PositionVelocityPairStateEci, Force6dEci> for J2StatePairEci {
    fn calc_force(&self, state_eci: &PositionVelocityPairStateEci) -> Force6dEci {
        let state_vec: Vec<PositionVelocityStateEci> = state_eci.convert();
        let state_chief = &state_vec[0];
        let state_deputy = &state_vec[1];

        let force_chief = Force3dLvlh::form_from_array(self.calc_force_(
            state_chief.position(),
            state_chief,
        ));
        let force_deputy = Force3dLvlh::form_from_array(self.calc_force_(
            state_deputy.position(),
            state_deputy,
        ));

        vec![force_chief, force_deputy].convert(state_chief)
    }
}
