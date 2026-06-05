use serde::Deserialize;

pub struct Bravo7InvKinematics {
    
}

type Quaternion = Vec<f32>;
type XYZ = Vec<f32>;

#[derive(Debug, Deserialize)]
pub struct BravoTwist {
    pose: XYZ,
    quat: Quaternion,
}

impl Bravo7InvKinematics {
    pub fn new() -> Self {
        Self {}
    }
}