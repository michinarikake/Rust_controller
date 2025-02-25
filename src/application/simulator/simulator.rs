use std::marker::PhantomData;

use crate::models::dynamics::propagator::Propagator;
use crate::models::dynamics::dynamics_trait::ContinuousDynamics;
use crate::models::state::state_trait::StateVector;
use crate::models::force::force_trait::Force;
use crate::models::disturbance::disturbance_trait::DisturbanceCalculator;

pub struct Simulator<T, U, P, D>
where
    T: StateVector + Clone,
    U: Force + Clone,
    P: Propagator<T, U>,
    D: ContinuousDynamics<T, U>,
{
    propagator: P,
    dynamics: D,
    state: T,
    disturbances: Vec<Box<dyn DisturbanceCalculator<T, U>>>,
    dt: f64,
    pub step: i64,
    pub t: f64,
    _marker: PhantomData<U>,
}

impl<T, U, P, D> Simulator<T, U, P, D>
where
    T: StateVector + Clone,
    U: Force,
    P: Propagator<T, U>,
    D: ContinuousDynamics<T, U>,
{
    pub fn new(propagator: P, dynamics: D, initial_state: T, dt: f64, step: i64, t0: f64) -> Self {
        Self {
            propagator,
            dynamics,
            state: initial_state,
            dt,
            step,
            disturbances: Vec::new(),
            t: t0,
            _marker: PhantomData,
        }
    }

    pub fn update(&mut self, input: &U) {
        let sum = input.add_vec(&self.calc_disturbance());
        self.state = self.propagator.propagate_continuous(&self.state, &sum, &self.dynamics, self.dt);
        self.t += self.dt;
    }

    pub fn add_disturbance(&mut self, disturbance: Box<dyn DisturbanceCalculator<T, U>>) {
        self.disturbances.push(disturbance);
    }

    fn calc_disturbance(&self) -> U {
        self.disturbances.iter()
            .fold(
                U::zeros(), 
                |acc, disturbance| 
                acc.add_vec(&disturbance.calc_force(&self.state))
            )
    }
    

    pub fn get_state(&self) -> &T {
        &self.state
    }
}

// #[cfg(test)]
// use crate::models::dynamics::dynamics_hcw::HcwDynamics;
// #[cfg(test)]
// use crate::models::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
// #[cfg(test)]
// use crate::models::force::force_3d_lvlh::Force3dLvlh;
// #[cfg(test)]
// use crate::models::dynamics::propagator::RungeKutta4Propagator;

// #[test]
// fn test_hcw_dynamics_with_rk4() {
//     let initial_state = PositionVelocityStateLvlh::form_from_list(
//         [100.0, 200.0, 300.0],
//         [0.1, 0.2, 0.3],
//     );
//     let external_force = Force3dLvlh::form_from_list([0.0, 0.0, 0.0]);
//     let dynamics = HcwDynamics::new(398600.4418, 7000.0);
//     let propagator = RungeKutta4Propagator;
//     let dt = 60.0; // タイムステップ（秒）

//     let mut simulator = Simulator::new(propagator, dynamics, initial_state, dt, 1000, 0.0);

//     for _ in 0..10 {
//         simulator.update(&external_force);
//     }

//     let final_state = simulator.get_state().get_vector();

//     // 期待値を手計算または理論値と比較する
//     assert!(final_state[0] > 0.0);
// }