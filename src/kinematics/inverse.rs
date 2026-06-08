use serde::Deserialize;
use crate::kinematics::utils::{
    Quaternion, Hz
};

pub struct Bravo7InvKinematics {
    
}


type XYZ = Vec<f32>;

#[derive(Debug, Deserialize)]
pub struct Twist {
    pose: XYZ,
    quat: Quaternion,
    sample_rate: Hz
}

impl Bravo7InvKinematics {
    pub fn new() -> Self {
        Self {}
    }
}