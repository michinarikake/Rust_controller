use std::any::Any;
use std::ops::{Add, Sub, Div, Mul};

#[allow(unused)]
use crate::domain::disturbance::air_drag_disturbance::{AirDragStateEci, AirDragStatePairEci, Surface};
#[allow(unused)]
use crate::domain::disturbance::j2_disturbance::{J2StateEci, J2StatePairEci};
use crate::domain::dynamics::dynamics_trait::ContinuousDynamics;
use crate::domain::dynamics::propagator::Propagator;
use crate::application::simulator::simulator::Simulator;
use crate::domain::state::state_trait::StateVector;
use crate::domain::force::force_trait::Force;
use crate::infrastructure::factory::initialization_wrapper::{InitializeState, InitializeDynamics, DisturbanceInitializer};

#[derive(Debug)]
pub enum InitializationTypeEnum {
    PositionVelocity,
    OrbitalElements,
    RelativePositionVelocity,
}

#[derive(Debug, Clone)]
pub enum DisturbanceEnum{
    J2,
    AirDrag,
}

#[derive(Debug)]
pub struct SimulationConfig{
    pub initialization: InitializationTypeEnum,
    pub init_data: Vec<f64>,
    pub constants: SimulationConstants,
    pub disturbances:Vec<DisturbanceEnum>,
}

#[derive(Debug)]
pub struct SimulationConstants {
    pub dt: f64,        // Time step (s)
    pub step: i64,        // Time step (s)
    pub t0: f64,         // Time start (s)
    pub a: f64,    // Reference semi-major axis (m)
    pub molecular_weight_chief: f64,
    pub wall_temperature_chief: f64,
    pub molecular_temperature: f64,
    pub mass_chief: f64,
    pub surfaces_chief: Vec<Surface>,
    pub molecular_weight_deputy: f64,
    pub wall_temperature_deputy: f64,
    pub mass_deputy: f64,
    pub surfaces_deputy: Vec<Surface>,
}

pub struct SimulatorFactory;

impl SimulatorFactory {
    pub fn create_simulator<T, U, P, D>(
        config: &SimulationConfig,
    ) -> Box<dyn Any>
    where 
        T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T> + Clone + InitializeState + DisturbanceInitializer<T, U> + 'static,
        U: Force + Add<Output = U> + Sub<Output = U> + Mul<f64, Output = U> + Div<f64, Output = U> + Clone + 'static,
        P: Propagator<T, U> + 'static,
        D: ContinuousDynamics<T, U> + InitializeDynamics + 'static,
    {
        let propagator = P::new(std::marker::PhantomData::<(T, U)>);
        let dynamics = D::initialize(&config);

        let state = T::initialize(&config);  // T に応じた初期化を呼び出す

        let simulator: Simulator<T, U, P, D> = Simulator::new(propagator, dynamics, state, config.constants.dt, config.constants.step, config.constants.t0);
        Box::new(SimulatorFactory::add_disturbance(simulator, config))
    }

    fn add_disturbance<T, U, P, D>(
        mut simulator: Simulator<T, U, P, D>,
        config: &SimulationConfig,
    ) -> Simulator<T, U, P, D>
    where
        T: StateVector + Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T> + Clone + InitializeState + DisturbanceInitializer<T, U>,
        U: Force + Add<Output = U> + Sub<Output = U> + Mul<f64, Output = U> + Div<f64, Output = U> + Clone,
        P: Propagator<T, U>, 
        D: ContinuousDynamics<T, U> + InitializeDynamics,
    {
        T::initialize_disturbances(config, &mut simulator);
        simulator
    }
    
}
