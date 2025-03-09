use std::env;
use std::process::Command;

use satellite_simulator::domain::controller::controller_trait::Controller;
use satellite_simulator::domain::force::force_converter::ForceConverter;
use satellite_simulator::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use satellite_simulator::domain::state::state_converter::StateConverter;
use satellite_simulator::infrastructure::factory::mode_scheduler_factory::ControllerFactory;
use satellite_simulator::infrastructure::settings::mode_shcedule_settings::{default_mode_scheduler_config, ControllerForceType, ControllerStateType, ControllerPropagatorType, ControllerDynamicsType};
use satellite_simulator::infrastructure::settings::simulation_config::default_simulation_config;
use satellite_simulator::infrastructure::factory::simulator_factory::SimulatorFactory;
use satellite_simulator::infrastructure::logger::logger::Logger;
use satellite_simulator::application::simulator::simulator::Simulator;
use satellite_simulator::infrastructure::settings::simulation_config::{StateType, ForceType, PropagatorType, DynamicsType};

fn main() {
    let log_filename = "simulation_log.csv";
    let log_filename2 = "controller_log.csv";
    let date_str = chrono::Local::now().format("%Y-%m-%d-%H-%M").to_string();
    
    // ロガーの作成
    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");
    let mut logger2 = Logger::new(log_filename2).expect("Failed to initialize logger");

    let config = default_simulation_config();
    let controller_config = default_mode_scheduler_config(&config);

    let mut simulator_box = SimulatorFactory::create_simulator::<StateType, ForceType, PropagatorType, DynamicsType>(&config);
    let simulator = simulator_box
        .downcast_mut::<Simulator<StateType, ForceType, PropagatorType, DynamicsType>>()
        .expect("Failed to cast Box<dyn Any> to Simulator");

    let mode_scheduler = ControllerFactory::<ControllerStateType, StateType, ControllerForceType, ControllerPropagatorType, ControllerDynamicsType>::create_mode_scheduler(simulator.get_state(), &config, &controller_config);

    // シミュレーション実行
    for _ in 0..simulator.step {
        let x: ControllerStateType = simulator.get_state().clone().convert();
        let external_force = mode_scheduler.compute_control_input(&x, simulator.t);
        let state_eci_vec: Vec<PositionVelocityStateEci> = simulator.get_state().clone().convert();
        let force_6d_eci = &external_force.convert(&state_eci_vec[0].clone());
        simulator.update(&force_6d_eci);

        // 同じタイムステップのデータを一行にまとめる
        logger.add_entry(simulator.get_state().clone());
        logger.add_entry(force_6d_eci.clone());
        logger2.add_entry(external_force.clone());
        logger2.add_entry(mode_scheduler.get_optimized_state_schedule(simulator.t).clone());

        logger.log(simulator.t);
        logger2.log(simulator.t);
    }

    logger.flush();
    logger2.flush();

    let log_filename = format!("data/{}/{}_simulation_log.csv", date_str, date_str);
    let log_filename2 = format!("data/{}/{}_controller_log.csv", date_str, date_str);
    
     // **プロジェクトのルートパスを取得**
     let current_dir = env::current_dir().expect("Failed to get current directory");
     let script_path = current_dir.join("src/plots/plot_common.py");
 
     // **スクリプトの存在を確認**
     if script_path.exists() {
         let output = Command::new("python")
             .arg(script_path.to_str().unwrap())
             .arg(&log_filename)  
             .output()
             .expect("Failed to execute plot_common.py");
        Command::new("python")
            .arg(script_path.to_str().unwrap())
            .arg(&log_filename2)  
            .output()
            .expect("Failed to execute plot_common.py");
 
         if output.status.success() {
             println!("Python script executed successfully.");
             println!("{}", String::from_utf8_lossy(&output.stdout));
         } else {
             eprintln!("Error executing Python script:\n{}", String::from_utf8_lossy(&output.stderr));
         }
     } else {
         eprintln!("Error: plot_common.py not found at {:?}", script_path);
     }
}