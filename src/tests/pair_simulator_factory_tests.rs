#[cfg(test)]
use crate::settings::simulation_config::default_simulation_config;
#[cfg(test)]
use crate::factory::simulator_factory::SimulatorFactory;
#[cfg(test)]
use crate::domain::force::force_3d_eci::Force3dEci;
#[cfg(test)]
use crate::repositry::logger::Logger;

#[test]
fn pair_state_simulation_test() {
    let log_filename = "test_pair_state_simulation_log.csv";

    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

    let config = default_simulation_config();
    let external_force = Force3dEci::form_from_list([0.0, 0.0, 0.0]);

    let mut simulator = SimulatorFactory::create_simulator(&config);

    for _ in 0..simulator.step {
        simulator.update(&external_force);

        logger.add_entry(simulator.get_state().clone());
        logger.add_entry(external_force.clone());

        logger.log(simulator.t);
    }

    logger.flush();
    logger.execute_python_script()
}
