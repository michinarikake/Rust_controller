use satellite_simulator::settings::simulation_config::default_simulation_config;
use satellite_simulator::factory::simulator_factory::SimulatorFactory;
use satellite_simulator::domain::force::force_3d_eci::Force3dEci;
use satellite_simulator::repositry::logger::Logger;

fn main() {
    let log_filename = "simulation_log.csv";

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