use ndarray::concatenate;

use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::math::formulations::Math;
use super::force_trait::Force;
use super::force_3d_eci::Force3dEci;
use super::force_3d_lvlh::Force3dLvlh;
use super::force_6d_eci::Force6dEci;
use super::force_6d_lvlh::Force6dLvlh;


pub trait ForceConverter<T> {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> T;
}

impl ForceConverter<Force3dLvlh> for Force3dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dLvlh {
        let force = Math::mat_eci2lvlh(&state_eci.position(), &state_eci.velocity()) * self;
        Force3dLvlh::form_from_array(force.get_vector().clone())
    }
}

impl ForceConverter<Force3dEci> for Force3dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        let force = Math::mat_lvlh2eci(&state_eci.position(), &state_eci.velocity()) * self;
        Force3dEci::form_from_array(force.get_vector().clone())
    }
}

#[allow(unused)]
impl ForceConverter<Force6dEci> for Vec<Force3dEci> {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dEci {
        let chief  = self[0].get_vector().clone();
        let deputy = self[1].get_vector().clone();
        Force6dEci::form_from_array(concatenate![ndarray::Axis(0), chief, deputy])
    }
}

#[allow(unused)]
impl ForceConverter<Force6dLvlh> for Vec<Force3dLvlh> {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dLvlh {
        let chief  = self[0].get_vector().clone();
        let deputy = self[1].get_vector().clone();
        Force6dLvlh::form_from_array(concatenate![ndarray::Axis(0), chief, deputy])
    }
}

impl ForceConverter<Force6dEci> for Vec<Force3dLvlh> {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dEci {
        let chief_eci: Force3dEci = self[0].convert(state_eci);
        let deputy_eci: Force3dEci = self[0].convert(state_eci);
        vec![chief_eci, deputy_eci].convert(state_eci)
    }
}

impl ForceConverter<Force6dLvlh> for Vec<Force3dEci> {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dLvlh {
        let chief_eci: Force3dLvlh = self[0].convert(state_eci);
        let deputy_eci: Force3dLvlh = self[0].convert(state_eci);
        vec![chief_eci, deputy_eci].convert(state_eci)
    }
}

#[allow(unused)]
impl ForceConverter<Vec<Force3dLvlh>> for Force6dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Vec<Force3dLvlh> {
        let chief = Force3dLvlh::form_from_array(self.chief());
        let deputy = Force3dLvlh::form_from_array(self.deputy());
        vec![chief, deputy]
    }
}

#[allow(unused)]
impl ForceConverter<Vec<Force3dEci>> for Force6dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Vec<Force3dEci> {
        let chief = Force3dEci::form_from_array(self.chief());
        let deputy = Force3dEci::form_from_array(self.deputy());
        vec![chief, deputy]
    }
}

impl ForceConverter<Vec<Force3dEci>> for Force6dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Vec<Force3dEci> {
        let chief: Force3dEci = Force3dLvlh::form_from_array(self.chief()).convert(state_eci);
        let deputy: Force3dEci = Force3dLvlh::form_from_array(self.deputy()).convert(state_eci);
        vec![chief, deputy]
    }
}

impl ForceConverter<Vec<Force3dLvlh>> for Force6dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Vec<Force3dLvlh> {
        let chief: Force3dLvlh = Force3dEci::form_from_array(self.chief()).convert(state_eci);
        let deputy: Force3dLvlh = Force3dEci::form_from_array(self.deputy()).convert(state_eci);
        vec![chief, deputy]
    }
}

impl ForceConverter<Force6dLvlh> for Force6dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dLvlh {
        let vec: Vec<Force3dLvlh> = self.convert(state_eci);
        vec.convert(state_eci)
    }
}

impl ForceConverter<Force6dEci> for Force6dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dEci {
        let vec: Vec<Force3dEci> = self.convert(state_eci);
        vec.convert(state_eci)
    }
}


#[allow(unused)]
impl ForceConverter<Force3dLvlh> for Force3dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dLvlh {
        self.clone()
    }
}

#[allow(unused)]
impl ForceConverter<Force3dEci> for Force3dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        self.clone()
    }
}

#[allow(unused)]
impl ForceConverter<Force6dEci> for Force6dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dEci {
        self.clone()
    }
}

#[allow(unused)]
impl ForceConverter<Force6dLvlh> for Force6dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force6dLvlh {
        self.clone()
    }
}