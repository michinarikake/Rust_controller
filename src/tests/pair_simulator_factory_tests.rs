#[cfg(test)]
use crate::settings::simulation_config::default_simulation_config;
#[cfg(test)]
use crate::factory::simulator_factory::SimulatorFactory;
#[cfg(test)]
use crate::repositry::logger::Logger;
#[cfg(test)]
use crate::application::simulator::simulator::Simulator;
#[cfg(test)]
use crate::settings::simulation_config::{StateType, ForceType, PropagatorType, DynamicsType};

#[test]
fn pair_state_simulation_test() {
    let log_filename = "test_pair_state_simulation_log.csv";

    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

    let config = default_simulation_config();
    let external_force = ForceType::form_from_list([0.0, 0.0, 0.0]);

    let mut simulator_box = SimulatorFactory::create_simulator::<StateType, ForceType, PropagatorType, DynamicsType>(&config);
    let simulator = simulator_box
        .downcast_mut::<Simulator<StateType, ForceType, PropagatorType, DynamicsType>>()
        .expect("Failed to cast Box<dyn Any> to Simulator");

    for _ in 0..simulator.step {
        simulator.update(&external_force);

        logger.add_entry(simulator.get_state().clone());
        logger.add_entry(external_force.clone());

        logger.log(simulator.t);
    }

    logger.flush();
    logger.execute_python_script()
}
