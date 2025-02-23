use ndarray::{Array1, Array2, arr1, arr2, concatenate, Axis, s};
use ndarray_linalg::Inverse;
use crate::application::simulator::simulator::Simulator;
use crate::domain::state::state_trait::StateVector;
use crate::repositry::logger::Logger;
use crate::domain::dynamics::propagator::RungeKutta4Propagator;
use crate::domain::dynamics::dynamics_2sat_2body::PairTwoBodyDynamics;
use crate::domain::state::position_velocity_pair_state_eci::PositionVelocityPairStateEci;
use crate::domain::state::orbital_elements::OrbitalElements;
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::force::force_3d_eci::Force3dEci;
use chrono::Local;
use std::process::Command;
use std::env;


#[test]
fn pair_state_simulation_test() {
    let log_filename = "pair_state_simulation_log.csv";

    // ロガーの作成
    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

    // 初期状態
    let mu = 3.986004 * 10f64.powi(14);
    let oe1 = OrbitalElements::form_from_elements(6928000.0, 0.001, 1.57079633, 0.0, 0.28869219, 0.0).unwrap();
    let oe2 = OrbitalElements::form_from_elements(6928010.0, 0.001, 1.57079633, 0.0, 0.28869219, 0.0).unwrap();
    let initial_state1 = PositionVelocityStateEci::form_from_orbital_elements(&oe1, mu);
    let initial_state2 = PositionVelocityStateEci::form_from_orbital_elements(&oe2, mu);
    println!("{}", initial_state1.get_vector());
    println!("{}", initial_state2.get_vector());
    let initial_state = PositionVelocityPairStateEci::form_from_state(initial_state1, initial_state2);
    let external_force = Force3dEci::form_from_list([0.0, 0.0, 0.0]);
    let dynamics = PairTwoBodyDynamics::new(mu);
    let propagator = RungeKutta4Propagator;
    let dt = 1.0;

    let mut simulator = Simulator::new(propagator, dynamics, initial_state.clone(), dt);

    // シミュレーション実行
    for step in 0..10000 {
        simulator.update(&external_force);

        // 同じタイムステップのデータを一行にまとめる
        logger.add_entry(simulator.get_state().clone());
        logger.add_entry(external_force.clone());

        logger.log(step);
    }

    logger.flush();

    let date_str = chrono::Local::now().format("%Y-%m-%d").to_string();
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
