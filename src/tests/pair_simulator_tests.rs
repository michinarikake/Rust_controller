#[cfg(test)]
use crate::application::simulator::simulator::Simulator;
#[cfg(test)]
use crate::domain::state::state_trait::StateVector;
#[cfg(test)]
use crate::infrastructure::logger::logger::Logger;
#[cfg(test)]
use crate::domain::dynamics::propagator::RungeKutta4Propagator;
#[cfg(test)]
use crate::domain::dynamics::dynamics_2sat_2body::PairTwoBodyDynamics;
#[cfg(test)]
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
#[cfg(test)]
use crate::domain::state::orbital_elements::OrbitalElements;
#[cfg(test)]
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
#[cfg(test)]
use crate::domain::force::force_6d_eci::Force6dEci;
#[cfg(test)]
use std::process::Command;
#[cfg(test)]
use std::env;
#[cfg(test)]
use crate::domain::state::state_converter::StateConverter;



#[test]
fn pair_state_simulation_test() {
    let log_filename = "pair_state_simulation_log.csv";

    // ロガーの作成
    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

    // 初期状態
    let oe1 = OrbitalElements::form_from_elements(6928000.0, 0.001, 1.57079633, 0.0, 0.28869219, 0.0).unwrap();
    let oe2 = OrbitalElements::form_from_elements(6928000.0, 0.001, 1.57079733, 0.0, 0.28869219, 0.0).unwrap();
    let initial_state1: PositionVelocityStateEci = oe1.convert();
    let initial_state2: PositionVelocityStateEci = oe2.convert();
    println!("{}", initial_state1.get_vector());
    println!("{}", initial_state2.get_vector());
    let initial_state: PositionVelocityPairStateEci = vec![initial_state1, initial_state2].convert();
    let external_force = Force6dEci::form_from_list([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let dynamics = PairTwoBodyDynamics::new();
    let propagator = RungeKutta4Propagator;
    let dt = 0.1;

    let mut simulator = Simulator::new(propagator, dynamics, initial_state.clone(), dt, 10000, 0.0);

    // シミュレーション実行
    for step in 0..100000 {
        simulator.update(&external_force);

        // 同じタイムステップのデータを一行にまとめる
        logger.add_entry(simulator.get_state().clone());
        logger.add_entry(external_force.clone());

        logger.log(step as f64 * dt);
    }

    logger.flush();

    let date_str = chrono::Local::now().format("%Y-%m-%d-%H-%M").to_string();
    let log_filename = format!("data/{}/{}_pair_state_simulation_log.csv", date_str, date_str);
    
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
