use super::disturbance_trait::DisturbanceCalculator;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::state::orbital_elements::OrbitalElements;
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::force::force_3d_lvlh::Force3dLvlh;

pub struct J2Disturbance {
    j2: f64, 
    mu: f64, 
    radius: f64
}

impl J2Disturbance {
    pub fn new(j2: f64, mu: f64, radius: f64) -> Self {
        Self { j2, mu, radius}
    }
}

impl DisturbanceCalculator for J2Disturbance {
    fn calc_force(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        let oe = OrbitalElements::form_from_state(state_eci, self.mu).unwrap();
        let r = state_eci.position_norm();
        let factor = 3.0 * self.mu * self.j2 * self.radius.powi(2) / (2.0 * r.powi(4));

        let theta = oe.nu_rad + oe.omega_rad;

        let fx = factor * (3.0 * theta.sin().powi(2) * oe.i_rad.sin().powi(2) - 1.0);
        let fy = -factor * (2.0 * theta).sin() * oe.i_rad.sin().powi(2);
        let fz = -factor * theta.sin() * (2.0 * oe.i_rad).sin();


        let f_lvlh = Force3dLvlh::form_from_list([fx, fy, fz]);
        Force3dEci::form_from_lvlh(&f_lvlh, state_eci)
    }
}