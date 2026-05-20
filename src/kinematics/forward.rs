const SERVO_NUM: usize = 6;

struct So100FwdKinematics {
    servo_theta: Vec<f32>,
    pose_twist: Vec<f32>,
}

impl So100FwdKinematics {
    pub fn new() -> Self {
        Self {
            servo_theta: Vec::with_capacity(SERVO_NUM),
            pose_twist: Vec::with_capacity(SERVO_NUM),
        }
    }

    pub fn update_vec_theta(&mut self, servo_theta: Vec<f32>) {
        assert_eq!(servo_theta.len(), SERVO_NUM);
        self.servo_theta = servo_theta;
    }

    pub fn update_single_theta(&mut self, joint_id: usize, servo_theta: f32) {
        self.servo_theta[joint_id] = servo_theta;
    }

    fn update_pose_twist(&mut self) {

    }
}
