use ndarray::{Array1};

/// **共通の状態ベクトルトレイト**
pub trait StateVector: Clone {
    fn get_vector(&self) -> &Array1<f64>;
    fn from_from_array(vec: Array1<f64>) -> Self;

    /// **加算 (ベクトル + ベクトル)**
    fn add_vec(self, rhs: Self) -> Self {
        Self::from_from_array(self.get_vector() + rhs.get_vector())
    }

    /// **減算 (ベクトル - ベクトル)**
    fn sub_vec(self, rhs: Self) -> Self {
        Self::from_from_array(self.get_vector() - rhs.get_vector())
    }

    /// **スカラー乗算 (ベクトル * スカラー)**
    fn mul_scalar(self, scalar: f64) -> Self {
        Self::from_from_array(self.get_vector() * scalar)
    }

    /// **スカラー除算 (ベクトル / スカラー)**
    fn div_scalar(self, scalar: f64) -> Self {
        Self::from_from_array(self.get_vector() / scalar)
    }
}
