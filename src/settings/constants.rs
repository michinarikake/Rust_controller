pub struct Constants {
    pub mu: f64,
    pub boltzmann_constant: f64,
    pub radius: f64,
    pub j2: f64,
}

pub static CONSTANTS: Constants = Constants {
    mu: 3.986004 * 10.0e14,
    boltzmann_constant: 1.380649e-23,
    radius: 6378.1e3,
    j2: 1.08263e-3,
};