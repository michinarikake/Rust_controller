use satellite_simulator::application::simulator::simulator::Simulator;
use satellite_simulator::repositry::logger::Logger;
use satellite_simulator::domain::dynamics::propagator::RungeKutta4Propagator;
use satellite_simulator::domain::dynamics::dynamics_hcw::HcwDynamics;
use satellite_simulator::domain::state::relative_position_velocity_state::RelativePositionVelocityState;
use satellite_simulator::domain::force::force_3d::Force3D;
use chrono::Local;

fn main() {
    let log_filename = "simulation_log.csv";
    
    // ロガーの作成
    let mut logger = Logger::new(log_filename).expect("Failed to initialize logger");

    // 初期状態
    let initial_state = RelativePositionVelocityState::form_from_list(
        [100.0, 200.0, 300.0],
        [0.1, 0.2, 0.3],
    );
    let external_force = Force3D::form_from_list([0.0, 0.0, 0.0]);
    let dynamics = HcwDynamics::new(398600.4418, 7000.0);
    let propagator = RungeKutta4Propagator;
    let dt = 10.0;

    let mut simulator = Simulator::new(propagator, dynamics, initial_state.clone(), dt);

    // シミュレーション実行
    for step in 0..1000 {
        simulator.update(&external_force);

        // 同じタイムステップのデータを一行にまとめる
        logger.add_entry(simulator.get_state().clone());
        logger.add_entry(external_force.clone());

        logger.log(step);
    }

    logger.flush();

    println!(
        "Simulation logs saved in: data/{}/{}_simulation_log.csv",
        Local::now().format("%Y-%m-%d"),
        Local::now().format("%Y-%m-%d")
    );
}