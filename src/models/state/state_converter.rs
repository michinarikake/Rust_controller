use crate::models::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::models::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::models::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::models::state::position_velocity_covariance_state_lvlh::PositionVelocityCovarianceStateLvlh;
use crate::models::state::orbital_elements::OrbitalElements;
use crate::models::state::state_trait::StateVector;
use crate::models::math::formulations::Math;
use ndarray::{arr1, concatenate, s, Axis};
use std::f64::consts::PI;
use crate::settings::constants::CONSTANTS;


pub trait StateConverter<T> {
    fn convert(&self) -> T;
}

#[allow(non_snake_case)]
impl StateConverter<OrbitalElements> for PositionVelocityStateEci {
    fn convert(&self) -> OrbitalElements {
        let mu = CONSTANTS.mu;
        let r = self.position();
        let r_norm = r.dot(&r).sqrt();
        let v = self.velocity();
        let h = Math::cross_product(&r, &v);
        let h_norm = h.dot(&h).sqrt();

        let n = arr1(&[-h[1], h[0], 0.0]);
        let n_norm = n.dot(&n).sqrt();

        let p = Math::cross_product(&v, &h) - mu * r.clone() / r_norm;
        let p_norm = p.dot(&p).sqrt();

        let q = Math::cross_product(&h, &p);
        let q_norm = q.dot(&q).sqrt();

        let e = p_norm / mu;

        let i_rad = (h[2] / h_norm).acos();
        let Omega_rad = (n[1]).atan2(n[0]);
        
        let mut cos_omega = n.dot(&p) / (n_norm * p_norm);
        if cos_omega > 1.0 { cos_omega = 1.0; }
        let mut omega_rad = cos_omega.acos();

        if p[2] < 0.0 { omega_rad = 2.0 * PI - omega_rad; }

        let mut nu_rad = r.dot(&(q / q_norm)).atan2(r.dot(&(&p / p_norm)));
        if r.dot(&v) < 0.0 { nu_rad = 2.0 * PI - nu_rad; }

        let a = 1.0 / (2.0 / r.dot(&r).sqrt() - v.dot(&v) / mu);

        OrbitalElements::form_from_elements(a, e, i_rad, omega_rad, Omega_rad, nu_rad).unwrap()
    }
}

impl StateConverter<PositionVelocityPairStateEci> for Vec<PositionVelocityStateEci> {
    fn convert(&self) -> PositionVelocityPairStateEci {
        let chief  = self[0].get_vector().clone();
        let deputy = self[1].get_vector().clone();
        PositionVelocityPairStateEci::form_from_array(concatenate![ndarray::Axis(0), chief, deputy])
    }
}


impl StateConverter<PositionVelocityStateEci> for OrbitalElements {
    fn convert(&self) -> PositionVelocityStateEci {
        let mu = CONSTANTS.mu;
        let p = self.a * (1.0 - self.e.powi(2));
        let r = p / (1.0 + self.e * self.nu_rad.cos());
        
        let r_pqw = arr1(&[r * self.nu_rad.cos(), r * self.nu_rad.sin(), 0.0]);
        let v_pqw = arr1(&[
            -((mu / p).sqrt()) * self.nu_rad.sin(),
            ((mu / p).sqrt()) * (self.e + self.nu_rad.cos()),
            0.0,
        ]);

        let rotation_matrix = Math::pqw_to_eci_matrix(self.i_rad, self.omega_rad, self.Omega_rad);

        let r_eci = rotation_matrix.dot(&r_pqw).to_vec();
        let v_eci = rotation_matrix.dot(&v_pqw).to_vec();
        let state = concatenate![ndarray::Axis(0), r_eci, v_eci];

        PositionVelocityStateEci::form_from_array(state)
    }
}

impl StateConverter<PositionVelocityPairStateEci> for Vec<OrbitalElements> {
    fn convert(&self) -> PositionVelocityPairStateEci {
        let chief: PositionVelocityStateEci = self[0].convert();
        let deputy: PositionVelocityStateEci = self[1].convert();
        vec![chief, deputy].convert()
    }
}

impl StateConverter<PositionVelocityStateLvlh> for PositionVelocityPairStateEci {
    fn convert(&self) -> PositionVelocityStateLvlh {
        let deputy_state = self.deputy();
        let chief_state = self.chief();
        let r_rel_eci = 
            deputy_state.slice(s![0..3]).to_owned() - chief_state.slice(s![0..3]).to_owned();
        let v_rel_eci = 
            deputy_state.slice(s![3..6]).to_owned() - chief_state.slice(s![3..6]).to_owned();

        let transform_matrix = Math::mat_eci2lvlh(
            &chief_state.slice(s![0..3]).to_owned(),
            &chief_state.slice(s![3..6]).to_owned(),
        );

        let omega = Math::cross_product(
            &chief_state.slice(s![0..3]).to_owned(),
            &chief_state.slice(s![3..6]).to_owned(),
        ) / chief_state.slice(s![0..3]).dot(&chief_state.slice(s![0..3]));

        let v_rel_corrected = v_rel_eci.to_owned() - Math::cross_product(&omega, &r_rel_eci.to_owned());

        let r_lvlh = transform_matrix.dot(&r_rel_eci.to_owned());
        let v_lvlh = transform_matrix.dot(&v_rel_corrected);

        PositionVelocityStateLvlh::form_from_array(concatenate![Axis(0), r_lvlh, v_lvlh])
    }
}

impl StateConverter<Vec<PositionVelocityStateEci>> for PositionVelocityPairStateEci {
    fn convert(&self) -> Vec<PositionVelocityStateEci> {
        let chief = PositionVelocityStateEci::form_from_array(self.chief());
        let deputy = PositionVelocityStateEci::form_from_array(self.deputy());
        vec![chief, deputy]
    }
}

impl StateConverter<Vec<OrbitalElements>> for PositionVelocityPairStateEci {
    fn convert(&self) -> Vec<OrbitalElements> {
        let chief = PositionVelocityStateEci::form_from_array(self.chief());
        let deputy = PositionVelocityStateEci::form_from_array(self.deputy());
        let chief_oe = chief.convert();
        let deputy_oe = deputy.convert();
        vec![chief_oe, deputy_oe]
    }
}


impl StateConverter<PositionVelocityStateEci> for PositionVelocityStateEci {
    fn convert(&self) -> PositionVelocityStateEci {
        self.clone()
    }
}

impl StateConverter<PositionVelocityPairStateEci> for PositionVelocityPairStateEci {
    fn convert(&self) -> PositionVelocityPairStateEci {
        self.clone()
    }
}

impl StateConverter<PositionVelocityStateLvlh> for PositionVelocityStateLvlh {
    fn convert(&self) -> PositionVelocityStateLvlh {
        self.clone()
    }
}

impl StateConverter<PositionVelocityCovarianceStateLvlh> for PositionVelocityCovarianceStateLvlh {
    fn convert(&self) -> PositionVelocityCovarianceStateLvlh {
        self.clone()
    }
}

impl StateConverter<OrbitalElements> for OrbitalElements {
    fn convert(&self) -> OrbitalElements {
        self.clone()
    }
}

