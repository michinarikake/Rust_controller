#[cfg(test)]
use crate::application::simulator::simulator::Simulator;
#[cfg(test)]
use crate::infrastructure::logger::logger::Logger;
#[cfg(test)]
use crate::domain::dynamics::propagator::RungeKutta4Propagator;
#[cfg(test)]
use crate::domain::dynamics::dynamics_hcw::HcwDynamics;
#[cfg(test)]
use crate::domain::state::relative_position_velocity_state_lvlh::PositionVelocityStateLvlh;
#[cfg(test)]
use crate::domain::force::force_3d_lvlh::Force3dLvlh;
#[cfg(test)]
use std::process::Command;
#[cfg(test)]
use std::env;

#[test]
fn simulator_log_test() {
    let log_filename = "simulation_log.csv";
    
    // ロガーの作成
    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

    // 初期状態
    let initial_state = PositionVelocityStateLvlh::form_from_list(
        [100.0, 200.0, 300.0],
        [0.1, 0.2, 0.3],
    );
    let external_force = Force3dLvlh::form_from_list([0.0, 0.0, 0.0]);
    let dynamics = HcwDynamics::new(7.000e6);
    let propagator = RungeKutta4Propagator;
    let dt = 10.0;

    let mut simulator = Simulator::new(propagator, dynamics, initial_state.clone(), dt, 10000, 0.0);

    // シミュレーション実行
    for step in 0..1000 {
        simulator.update(&external_force);

        // 同じタイムステップのデータを一行にまとめる
        logger.add_entry(simulator.get_state().clone());
        logger.add_entry(external_force.clone());

        logger.log(step as f64 * dt);
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