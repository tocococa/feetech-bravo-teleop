use serde::Deserialize;
use crate::kinematics::utils::{
    Quaternion, Hz
};

pub struct Bravo7InvKinematics {
    
}


type XYZ = Vec<f64>;

#[derive(Debug, Deserialize)]
pub struct Twist {
    pub pose: XYZ,
    pub  quat: Quaternion,
    pub sample_rate: Hz
}

impl Bravo7InvKinematics {
    pub fn new() -> Self {
        Self {}
    }
}