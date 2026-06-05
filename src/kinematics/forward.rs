use std::f32::consts::PI;
use std::time::{Instant};

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
    pub fn clone(&self) -> Self {
        Mat3::new(self.data)
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

fn quat_conjugate(q: &[f32]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

fn quat_multiply(a: &[f32], b: &[f32]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

pub struct So100FwdKinematics {
    joint_thetas: Vec<f32>,
    ee_rot: Mat3,
    ee_position: Vec<f32>,
    ee_rot_ref: Mat3,
    ee_pos_ref: Vec<f32>,
    params: DHParams,
    last_update: Instant,
    ee_rot_vel: Vec<f32>,
    ee_pos_vel: Vec<f32>,
}

const SERVO_NUM: usize = 6;

impl So100FwdKinematics {
    pub fn new() -> Self {
        Self {
            joint_thetas: vec![0.0; SERVO_NUM],
            ee_rot: Mat3::new([[0.0; 3]; 3]),
            ee_position: Vec::with_capacity(3),
            ee_rot_ref: Mat3::new([[0.0; 3]; 3]),
            ee_pos_ref: vec![0.0; 3],
            params: DHParams::default(),
            ee_rot_vel: Vec::with_capacity(4),
            ee_pos_vel: Vec::with_capacity(3),
            last_update: Instant::now()
        }
    }

    pub fn get_ee_position(&self) -> Vec<f32> {
        let mut pos = Vec::with_capacity(3);
        for i in 0..3 {
            pos.push(self.ee_position[i] - self.ee_pos_ref[i]);
        }
        pos
    }

    fn get_ee_rotation(&self, rot: Mat3) -> Vec<f32> {
        let r_rel = rot.transpose().mul(&self.ee_rot_ref);
        // convert rotation to quaternion
        let trace = r_rel.data[0][0] + r_rel.data[1][1] + r_rel.data[2][2];
        let mut q = [0.0; 4];
        if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            q[0] = 0.25 / s;
            q[1] = (r_rel.data[1][2] - r_rel.data[2][1]) * s;
            q[2] = (r_rel.data[2][0] - r_rel.data[0][2]) * s;
            q[3] = (r_rel.data[0][1] - r_rel.data[1][0]) * s;
        } else {
            if r_rel.data[0][0] > r_rel.data[1][1] && r_rel.data[0][0] > r_rel.data[2][2] {
                let s = 2.0 * (1.0 + r_rel.data[0][0] - r_rel.data[1][1] - r_rel.data[2][2]).sqrt();
                q[0] = (r_rel.data[1][2] - r_rel.data[2][1]) / s;
                q[1] = 0.25 * s;
                q[2] = (r_rel.data[1][0] + r_rel.data[0][1]) / s;
                q[3] = (r_rel.data[2][0] + r_rel.data[0][2]) / s;
            } else if r_rel.data[1][1] > r_rel.data[2][2] {
                let s = 2.0 * (1.0 + r_rel.data[1][1] - r_rel.data[0][0] - r_rel.data[2][2]).sqrt();
                q[0] = (r_rel.data[2][0] - r_rel.data[0][2]) / s;
                q[1] = (r_rel.data [1][0] + r_rel.data[0][1]) / s;
                q[2] = 0.25 * s;
                q[3] = (r_rel.data[2][1] + r_rel.data[1][2]) / s;
            } else {
                let s = 2.0 * (1.0 + r_rel.data[2][2] - r_rel.data[0][0] - r_rel.data[1][1]).sqrt();
                q[0] = (r_rel.data[0][1] - r_rel.data[1][0]) / s;
                q[1] = (r_rel.data[2][0] + r_rel.data[0][2]) / s;
                q[2] = (r_rel.data[2][1] + r_rel.data[1][2]) / s;
                q[3] = 0.25 * s;
            }
        }
        q.to_vec()
    }

    pub fn re_center_ref(&mut self) {
        self.ee_rot_ref = self.ee_rot.clone();
        self.ee_pos_ref = self.ee_position.clone();
        self.ee_rot_vel.clear();
        self.ee_pos_vel.clear();
        self.last_update = Instant::now();
    }

    pub fn update_theta(&mut self, joint_id: usize, joint_theta: f32) {
        self.joint_thetas[joint_id] = joint_theta;
    }

    fn compute_ee_velocities(&mut self, new_pos: Vec<f32>, new_quat: Vec<f32>) {
        let time_delta = self.last_update.elapsed().as_secs() as f32;
        let x_vel = (new_pos[0] - self.ee_position[0]) / time_delta;
        let y_vel = (new_pos[1] - self.ee_position[1]) / time_delta;
        let z_vel = (new_pos[2] - self.ee_position[2]) / time_delta;
        self.ee_pos_vel = vec![x_vel, y_vel, z_vel];

        let q_rel = quat_multiply(&new_quat, &quat_conjugate(&self.get_ee_rotation(self.ee_rot.clone())));
        let q_rel = if q_rel[0] < 0.0 {
            [-q_rel[0], -q_rel[1], -q_rel[2], -q_rel[3]]
        } else {
            q_rel
        };

        let w = q_rel[0].clamp(-1.0, 1.0);
        let angle = 2.0 * w.acos();
    
        // Small-angle handling
        if angle.abs() < 1e-6 {
            self.ee_rot_vel = vec![0.0; 3];
        }
        else {
            let sin_half_angle = (1.0 - w * w).sqrt();
            
            let axis = if sin_half_angle < 1e-6 {
                [1.0, 0.0, 0.0]
            } else {
                [
                    q_rel[1] / sin_half_angle,
                    q_rel[2] / sin_half_angle,
                    q_rel[3] / sin_half_angle,
                ]
            };
        
            let omega_mag = angle / time_delta;
        
            self.ee_rot_vel = vec![
                axis[0] * omega_mag,
                axis[1] * omega_mag,
                axis[2] * omega_mag,
            ];
        }
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

        let quat = self.get_ee_rotation(r.clone());
        self.compute_ee_velocities(pos.clone(), quat);
        
        self.ee_position = pos;
        self.ee_rot = r;
        
    }
}
