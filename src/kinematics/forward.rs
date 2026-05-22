use std::f32::consts::PI;

// as of May 2026, RFC #3681 "default_field_values" does not
// allow for default values for vectors, so we still need to use imp::Default
// for the SO-100 arm, we need a struct and then an implementation
// that loads the DH params into it

struct Mat3 {
    data: [[f32; 3]; 3],
}

impl Mat3 {
    pub fn new(data: [[f32; 3]; 3]) -> Self {
        Self { data }
    }
    fn transpose(&self) -> Self {
        let data = self.data;
        Mat3 {
            data: [
                [data[0][0], data[1][0], data[2][0]],
                [data[0][1], data[1][1], data[2][1]],
                [data[0][2], data[1][2], data[2][2]],
            ],
        }
    }
    fn mul(&self, other: &Mat3) -> Mat3 {
        let a = self.data;
        let b = other.data;

        let mut r = [[0.0; 3]; 3];

        for i in 0..3 {
            for j in 0..3 {
                r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        Mat3::new(r)
    }
}

struct DHParams {
    a: Vec<f32>,
    alpha: Vec<f32>,
    d: Vec<f32>,
    theta_offset: Vec<f32>,
}

impl Default for DHParams {
    fn default() -> Self {
        Self {
            a: vec![0.0425, 0.107, 0.116, 0.135, 0.0],
            alpha: vec![PI / 2.0, 0.0, 0.0, PI / 2.0, -PI / 2.0],
            d: vec![0.0165, 0.0, 0.0, 0.0, -0.06],
            theta_offset: vec![0.0, -1.8, PI / 2.0, -1.0, 0.0],
        }
    }
}

pub struct So100FwdKinematics {
    joint_thetas: Vec<f32>,
    ee_rot: Mat3,
    ee_position: Vec<f32>,
    params: DHParams,
}

const SERVO_NUM: usize = 6;

impl So100FwdKinematics {
    pub fn new() -> Self {
        Self {
            joint_thetas: vec![0.0; SERVO_NUM],
            ee_rot: Mat3::new([[0.0; 3]; 3]),
            ee_position: Vec::with_capacity(3),
            params: DHParams::default(),
        }
    }

    // pub fn update_vec_theta(&mut self, joint_thetas: Vec<f32>) {
    //     assert_eq!(joint_thetas.len(), SERVO_NUM);
    //     self.joint_thetas = joint_thetas;
    // }

    pub fn update_single_theta(&mut self, joint_id: usize, joint_theta: f32) {
        self.joint_thetas[joint_id] = joint_theta;
    }

    pub fn update_pose_twist(&mut self) {
        // this comes from math that was done by hand to
        // avoid constant matrix multiplications, and is based on the DH params
        // Note: q* and c* numbering is 1-indexed, but the joint_thetas vector
        // is 0-indexed so q1 = joint_thetas[0], etc.
        let (q1, q2, q3, q4, q5) = (
            self.joint_thetas[0] + self.params.theta_offset[0],
            self.joint_thetas[1] + self.params.theta_offset[1],
            self.joint_thetas[2] + self.params.theta_offset[2],
            self.joint_thetas[3] + self.params.theta_offset[3],
            self.joint_thetas[4] + self.params.theta_offset[4],
        );
        let c1 = (q1).cos();
        let s1 = (q1).sin();
        let c2 = (q2).cos();
        let s2 = (q2).sin();
        // let c3 = (q3).cos();
        // let s3 = (q3).sin();
        // let c4 = (q4).cos();
        // let s4 = (q4).sin();
        let c5 = (q5).cos();
        let s5 = (q5).sin();

        let c23 = (q2 + q3).cos();
        let s23 = (q2 + q3).sin();

        let c234 = (q2 + q3 + q4).cos();
        let s234 = (q2 + q3 + q4).sin();

        // ee position
        let x = c1
            * (self.params.a[0]
                + self.params.a[1] * c2
                + self.params.a[2] * c23
                + self.params.a[3] * c234)
            + c1 * self.params.d[4] * s234;
        let y = s1
            + (self.params.a[0]
                + self.params.a[1] * c2
                + self.params.a[2] * c23
                + self.params.a[3] * c234)
            + s1 * self.params.d[4] * s234;
        let z = self.params.d[0]
            + self.params.a[1] * s2
            + self.params.a[2] * s23
            + self.params.a[3] * s234
            - self.params.d[4] * c234;
        let pos = vec![x, y, z];

        // ee twist, the r[m,n] notation comes from the T_[05] rotation matrix for the SO-100
        let r11 = c1 * c234 * c5 + s1 * s5;
        let r12 = -c1 * s234;
        let r13 = -c1 * c234 * s5 + s1 * c5;

        let r21 = s1 * c234 * c5 - c1 * s5;
        let r22 = -s1 * s234;
        let r23 = -s1 * c234 * s5 - c1 * c5;

        let r31 = s234 * c5;
        let r32 = c234;
        let r33 = -s234 * s5;

        let r = Mat3::new([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]);

        self.ee_position = pos;
        self.ee_rot = r;
    }
}
