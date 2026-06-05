use serde::Deserialize;

pub struct Bravo7InvKinematics {
    
}

#[derive(Debug, Deserialize)]
pub struct BravoTwist {
    pose: Vec<f32>,
    quat: Vec<f32>,
}

impl Bravo7InvKinematics {
    pub fn new() -> Self {
        Self {}
    }
}