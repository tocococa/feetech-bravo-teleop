pub struct Bravo7InvKinematics {
    
}

impl Bravo7InvKinematics {
    pub fn new() -> Self {
        Self {}
    }

    pub fn solve_ik_clamped(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        // Solve inverse kinematics using SVD-based damped pseudo-inverse
        // with adaptive damping. (Should) handle singularities by scaling
        // damping based on singular values.
        // TODO: finish implementation
        let theta1 = y.atan2(x);
        let theta2 = (x * x + y * y).sqrt();

        Some((theta1, theta2))
    }
}