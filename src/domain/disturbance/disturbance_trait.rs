use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;

pub trait DisturbanceCalculator {
    fn calc_force(&self, state_eci: &PositionVelocityStateEci) -> Force3dEci;
}

// pub struct DisturbanceCalculator {
//     disturbances: Vec<Box<dyn Disturbance>>,
// }

// impl DisturbanceCalculator {
//     pub fn new(config: &DisturbanceConfig) -> Self {
//         let mut disturbances: Vec<Box<dyn Disturbance>> = Vec::new();
//         disturbances.push(Box::new(J2Disturbance::new(config)));
//         disturbances.push(Box::new(AirDragDisturbance::new(config)));

//         Self { disturbances }
//     }

//     pub fn calc_total_force(&self, position_eci: &Vector3<f64>, velocity_eci: &Vector3<f64>) -> Vector3<f64> {
//         self.disturbances.iter()
//             .map(|dist| dist.calc_force(position_eci, velocity_eci))
//             .sum()
//     }
// }
