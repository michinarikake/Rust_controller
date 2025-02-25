use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::math::formulations::Math;
use super::force_trait::Force;
use super::force_3d_eci::Force3dEci;
use super::force_3d_lvlh::Force3dLvlh;


pub trait ForceConverter<T> {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> T;
}

#[allow(non_snake_case)]
impl ForceConverter<Force3dLvlh> for Force3dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dLvlh {
        let force = Math::mat_eci2lvlh(&state_eci.position(), &state_eci.velocity()) * self;
        Force3dLvlh::form_from_array(force.get_vector().clone())
    }
}

#[allow(non_snake_case)]
impl ForceConverter<Force3dEci> for Force3dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        let force = Math::mat_lvlh2eci(&state_eci.position(), &state_eci.velocity()) * self;
        Force3dEci::form_from_array(force.get_vector().clone())
    }
}


impl ForceConverter<Force3dLvlh> for Force3dLvlh {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dLvlh {
        self.clone()
    }
}

impl ForceConverter<Force3dEci> for Force3dEci {
    fn convert(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci {
        self.clone()
    }
}