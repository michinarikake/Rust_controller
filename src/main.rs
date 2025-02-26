use satellite_simulator::domain::force::force_trait::Force;
use satellite_simulator::infrastructure::settings::simulation_config::default_simulation_config;
use satellite_simulator::infrastructure::factory::simulator_factory::SimulatorFactory;
use satellite_simulator::infrastructure::logger::logger::Logger;
use satellite_simulator::application::simulator::simulator::Simulator;
use satellite_simulator::infrastructure::settings::simulation_config::{StateType, ForceType, PropagatorType, DynamicsType};

fn main() {
    let log_filename = "test_pair_state_simulation_log.csv";

    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

    let config = default_simulation_config();
    let external_force = ForceType::zeros();

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
