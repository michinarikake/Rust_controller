use ndarray::Array1;

pub trait Force: Clone{
    fn get_vector(&self) -> &Array1<f64>;
    fn form_from_array(vec: Array1<f64>) -> Self;
    fn zeros() -> Self;

    /// **加算 (ベクトル + ベクトル)**
    fn add_vec(&self, rhs: &Self) -> Self {
        Self::form_from_array(self.get_vector() + rhs.get_vector())
    }

    /// **減算 (ベクトル - ベクトル)**
    fn sub_vec(&self, rhs: &Self) -> Self {
        Self::form_from_array(self.get_vector() - rhs.get_vector())
    }

    /// **スカラー乗算 (ベクトル * スカラー)**
    fn mul_scalar(&self, scalar: f64) -> Self {
        Self::form_from_array(self.get_vector() * scalar)
    }

    /// **スカラー除算 (ベクトル / スカラー)**
    fn div_scalar(&self, scalar: f64) -> Self {
        Self::form_from_array(self.get_vector() / scalar)
    }
}