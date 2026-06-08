// move Mat3 n things over here
// move Quat n things over here

//use std::ops::Add;

pub type Quaternion = Vec<f64>;
pub type Hz = f64;

pub fn quat_conjugate(q: Quaternion) -> Quaternion {
    vec![q[0], -q[1], -q[2], -q[3]]
}

pub fn quat_multiply(q: &Quaternion, r: &Quaternion) -> Quaternion {
    let (w0, x0, y0, z0) = (q[0], q[1], q[2], q[3]);
    let (w1, x1, y1, z1) = (r[0], r[1], r[2], r[3]);
    vec![
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ]
}

fn quat_normalize(q: &Quaternion) -> Quaternion {
    let norm = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    assert!(norm > 1e-10, "Cannot normalize a zero quaternion");
    q.iter().map(|x| x / norm).collect()
}

fn quat_scale(q: &Quaternion, s: f64) -> Quaternion {
    q.iter().map(|x| x * s).collect()
}

fn quat_add(q: &Quaternion, r: &Quaternion) -> Quaternion {
    q.iter().zip(r.iter()).map(|(a, b)| a + b).collect()
}

pub fn integrate_first_order(pose: &Quaternion, omega: &Vec<f64>, dt: f64) -> Quaternion {
    assert_eq!(pose.len(), 4, "Pose must be [w, x, y, z]");
    assert_eq!(omega.len(), 3, "Omega must be [wx, wy, wz]");

    // Angular velocity as a pure quaternion [0, wx, wy, wz]
    let omega_q = vec![0.0, omega[0], omega[1], omega[2]];

    // q_dot = 0.5 * (q ⊗ ω_q)
    let q_dot = quat_scale(&quat_multiply(pose, &omega_q), 0.5);

    // q_next = q + q_dot * dt
    let q_next = quat_add(pose, &quat_scale(&q_dot, dt));

    quat_normalize(&q_next)
}
