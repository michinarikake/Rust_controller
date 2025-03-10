use ndarray::{Array1, Array2};

/// **共通の状態ベクトルトレイト**
pub trait StateVector: Clone {
    fn get_vector(&self) -> &Array1<f64>;
    fn form_from_array(vec: Array1<f64>) -> Self;

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

    fn mul_mat(&self, mat: &Array2<f64>) -> Self {
        Self::form_from_array(self.get_vector().dot(mat))
    }

    // ここに演算子オーバーロードを実装したい...
    // traitの実装先が2つ以上あるとき、異なる型間の演算がぶつかってしまうからできないらしい
    // やるとしたらマクロかな...
}
