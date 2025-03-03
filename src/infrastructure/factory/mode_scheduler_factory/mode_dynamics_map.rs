use std::collections::HashMap;
use crate::domain::controller::mode_controller::mode_optimizer::{ModeId, ModeDynamicsMap, ContinuousDynamicsAndDifferentiable, CostAndDifferentiable};
use crate::domain::state::position_velocity_state_eci::PositionVelocityStateEci;
use crate::domain::force::force_3d_eci::Force3dEci;
use crate::domain::force::force_trait::Force;
use crate::domain::cost::quadric_cost::QuadraticCost;
use super::linear_controllerd_dynamics::LinearControlledDynamics;
use ndarray::{Array2, arr1};

/// **27通りの制御入力を考慮した `ModeDynamicsMap` を作成**
pub fn create_mode_dynamics_map() -> ModeDynamicsMap<PositionVelocityStateEci, Force3dEci> {
    let mut dynamics_mapping = HashMap::new();
    let mut cost_mapping = HashMap::new();

    let q_matrix = Array2::<f64>::eye(6);
    let r_matrix = Array2::<f64>::eye(3);
    let qf_matrix = Array2::<f64>::eye(6);
    
    // 制御入力の組み合わせ (-1,0,1) の 27 通り
    for (i, &ux) in [-1.0, 0.0, 1.0].iter().enumerate() {
        for (j, &uy) in [-1.0, 0.0, 1.0].iter().enumerate() {
            for (k, &uz) in [-1.0, 0.0, 1.0].iter().enumerate() {
                let mode_id = ModeId::new(i * 9 + j * 3 + k);
                let control_input = Force3dEci::form_from_array(arr1(&[ux, uy, uz]));
                let dynamics = LinearControlledDynamics::new(control_input.clone());
                let cost = QuadraticCost::new(q_matrix.clone(), r_matrix.clone(), qf_matrix.clone(), 100.0);

                dynamics_mapping.insert(mode_id, Box::new(dynamics) as Box<dyn ContinuousDynamicsAndDifferentiable<PositionVelocityStateEci, Force3dEci>>);
                cost_mapping.insert(mode_id, Box::new(cost) as Box<dyn CostAndDifferentiable<PositionVelocityStateEci, Force3dEci>>);
            }
        }
    }

    ModeDynamicsMap::new(dynamics_mapping, cost_mapping)
}
