use ndarray::{Array1, Array2, arr1, arr2};
use ndarray_linalg::Inverse;

pub struct Math {}

impl Math {
    /// **3次元ベクトルの外積を計算**
    pub fn cross_product(v1: &Array1<f64>, v2: &Array1<f64>) -> Array1<f64> {
        assert!(v1.len() == 3 && v2.len() == 3, "Both vectors must be of length 3.");

        arr1(&[
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0],
        ])
    }

    pub fn normalize(v: &Array1<f64>) -> Array1<f64> {
        let norm = v[0].powi(2) + v[1].powi(2) + v[2].powi(2);
        v / norm
    }


    pub fn mat_eci2lvlh(position: &Array1<f64>, velocity: &Array1<f64>) -> Array2<f64> {
        let r_norm = position.dot(position).sqrt();
        let h = Math::cross_product(position, velocity);
        let h_norm = h.dot(&h).sqrt();

        let r_hat = position / r_norm;
        let h_hat = &h / h_norm;
        let v_hat = Math::cross_product(&h_hat, &r_hat);

        arr2(&[
            [r_hat[0], r_hat[1], r_hat[2]],
            [v_hat[0], v_hat[1], v_hat[2]],
            [h_hat[0], h_hat[1], h_hat[2]],
        ])
    }

    pub fn mat_lvlh2eci(position: &Array1<f64>, velocity: &Array1<f64>) -> Array2<f64> {
        Self::mat_eci2lvlh(position, velocity).into_owned().inv().expect("Matrix inversion failed")
    }

    #[allow(non_snake_case)]
    pub fn pqw_to_eci_matrix(i_rad: f64, omega_rad: f64, Omega_rad: f64) -> Array2<f64> {
        let cos_Omega = Omega_rad.cos();
        let sin_Omega = Omega_rad.sin();
        let cos_i = i_rad.cos();
        let sin_i = i_rad.sin();
        let cos_omega = omega_rad.cos();
        let sin_omega = omega_rad.sin();

        Array2::from_shape_vec(
            (3, 3),
            vec![
                cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i,
                -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i,
                sin_Omega * sin_i,
                sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i,
                -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i,
                -cos_Omega * sin_i,
                sin_omega * sin_i,
                cos_omega * sin_i,
                cos_i,
            ],
        )
        .unwrap()
    }
}
