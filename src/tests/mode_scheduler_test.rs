use crate::domain::force::force_6d_eci;
use crate::domain::force::force_converter::ForceConverter;
use crate::domain::force::force_trait::Force;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
#[cfg(test)]
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
use crate::domain::state::state_trait::StateVector;
#[cfg(test)]
use crate::infrastructure::settings::simulation_config::default_simulation_config;
#[cfg(test)]
use crate::infrastructure::factory::simulator_factory::SimulatorFactory;
#[cfg(test)]
use crate::infrastructure::logger::logger::Logger;
#[cfg(test)]
use crate::application::simulator::simulator::Simulator;
#[cfg(test)]
use crate::infrastructure::settings::simulation_config::{StateType, ForceType, PropagatorType, DynamicsType};
#[cfg(test)]
use crate::infrastructure::settings::mode_shcedule_settings::{
    default_mode_scheduler_config, 
    ControllerDynamicsType, 
    ControllerForceType, 
    ControllerPropagatorType, 
    ControllerStateType};
#[cfg(test)]
use crate::infrastructure::factory::mode_scheduler_factory::ControllerFactory;
#[cfg(test)]
use crate::domain::controller::controller_trait::Controller;
#[cfg(test)]
use crate::domain::state::state_converter::StateConverter;
#[cfg(test)]
use std::env;
#[cfg(test)]
use std::process::Command;


#[test]
fn test_mode_scheduler_optimization() {
    let log_filename = "simulation_log.csv";
    
    // ロガーの作成
    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

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
        let force_6d_eci = &external_force.convert(&state_eci_vec[0].clone()); //FIXME: ここが状態量依存
        simulator.update(&force_6d_eci);

        // 同じタイムステップのデータを一行にまとめる
        logger.add_entry(simulator.get_state().clone());
        // logger.add_entry(force_6d_eci.clone());
        logger.add_entry(external_force.clone());

        logger.log(simulator.t);
    }

    logger.flush();

    let date_str = chrono::Local::now().format("%Y-%m-%d-%H-%M").to_string();
    let log_filename = format!("data/{}/{}_simulation_log.csv", date_str, date_str);
    
     // **プロジェクトのルートパスを取得**
     let current_dir = env::current_dir().expect("Failed to get current directory");
     let script_path = current_dir.join("src/plots/plot_common.py");
 
     // **スクリプトの存在を確認**
     if script_path.exists() {
         let output = Command::new("python") // または "python"（環境による）
             .arg(script_path.to_str().unwrap())
             .arg(&log_filename)  
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