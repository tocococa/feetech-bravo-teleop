use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use zmq;

use clap::Parser;
use serde::Deserialize;

use feetech_bravo_teleop::{
    Driver, ReadCommand::CurrentPosition, So100FwdKinematics, Twist, integrate_first_order,
};

use feetech_bravo_teleop::utils::step_to_rads;

#[derive(Parser, Debug)]
#[command(
    version, about, long_about = None,
    after_help = "
    Keep terminal in focus to use, press space to engage teleop mode, relese to disengage.\n
    Press space and G together to engage the gripper control.\n
    Remember to allow control of the serial port at which the Feetech connected to (e.g., sudo chmod 666 /dev/ttyACM0)"
)]
struct Cli {
    #[arg(short, long, default_value = "./calibration.json")]
    calibration_file: String,
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    port: String,
    #[arg(short, long, default_value = "false")]
    debug: bool,
    #[arg(short, long, default_value = "5555")]
    tcp_port: String,
}

#[derive(Debug, Deserialize, Clone)]
struct JointCalibration {
    id: u8,
    //drive_mode: u8, available but unused
    homing_offset: i32,
    //range_min: i32, ==
    //range_max: i32, ==
}

#[derive(Debug, Deserialize)]
struct JointState {
    calibration: JointCalibration,
    #[serde(default)]
    current_step: u16,
    #[serde(default)]
    current_rads: f32,
}

fn update_leader_state_serial_read(
    teleop_input: &mut Driver,
    fwd_kinematics: &mut So100FwdKinematics,
    servo_calib: &HashMap<String, JointCalibration>,
    echo: bool,
) -> HashMap<u8, JointState> {
    let mut servo_positions: Vec<u16> = [0; 6].to_vec();
    let mut servo_states: HashMap<u8, JointState> = servo_calib
        .values()
        .map(|calib| {
            (
                calib.id,
                JointState {
                    calibration: calib.clone(),
                    current_step: 0,
                    current_rads: 0.0,
                },
            )
        })
        .collect();
    for motor_id in 1u8..=6u8 {
        servo_positions[(motor_id - 1) as usize] =
            teleop_input.read(motor_id, CurrentPosition).unwrap();
        if let Some(servo_info) = servo_states.get_mut(&motor_id) {
            servo_info.current_step = servo_positions[(motor_id - 1) as usize];
            servo_info.current_rads = step_to_rads(
                servo_info.current_step as i32,
                servo_info.calibration.homing_offset,
            );
            fwd_kinematics.update_theta((motor_id - 1) as usize, servo_info.current_rads);
        }
    }
    if echo {
        println!("Servo states:\n{:?}", servo_states);
    }
    fwd_kinematics.update_pose_twist();
    return servo_states;
}

fn main() {
    let cli = Cli::parse();

    let json =
        std::fs::read_to_string(cli.calibration_file).expect("Failed to read calibration file");
    let servo_calib: HashMap<String, JointCalibration> =
        serde_json::from_str(&json).expect("Failed to parse calibration file");

    if cli.debug {
        for (joint_name, joint_info) in &servo_calib {
            println!("Joint: {}, Info: {:?}", joint_name, joint_info);
        }
    }

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_clone.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    if cli.debug {
        println!(
            "ZeroMQ server will start in tcp://localhost:{}",
            cli.tcp_port
        )
    }

    let context = zmq::Context::new();
    // socket type has different state machines
    let responder = context.socket(zmq::REP).unwrap();

    assert!(responder.bind(&format!("tcp://*:{}", cli.tcp_port)).is_ok());

    let mut teleop_input = Driver::new(&cli.port);

    let mut recenter = true;
    let mut fwd_kinematics = So100FwdKinematics::new();

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        // this is a blocking call
        let msg = responder.recv_string(0).unwrap().unwrap(); // there must be a smarter way to do this
        let bravo_twist: Twist = serde_json::from_str(&msg).expect("Failed to parse ee twist");
        if cli.debug {
            println!("Received data {:?}", bravo_twist);
        }

        let servo_states = update_leader_state_serial_read(
            &mut teleop_input,
            &mut fwd_kinematics,
            &servo_calib,
            cli.debug,
        );

        fwd_kinematics.compute_ee_velocities(bravo_twist.sample_rate);
        let (xyz_vel, omega_rot) = fwd_kinematics.get_ee_velocities();

        if cli.debug {
            println!("[SO-100] Current Servo Angles (rads):");
            for (servo_id, joint_info) in &servo_states {
                println!("{}: {:.4}", servo_id, joint_info.current_rads);
            }
            let ee_pos = fwd_kinematics.get_ee_position();
            println!("[SO-100] Current end effector position:");
            println!("x: {:?}, y: {:?}, z: {:?}", ee_pos[0], ee_pos[1], ee_pos[2]);
            println!("[SO-100] Current ee velocities");
            println!(
                "x: {:?}, y: {:?}, z: {:?} [?/s]",
                xyz_vel[0], xyz_vel[1], xyz_vel[2]
            );
            println!(
                "w_1: {:?}, w_2: {:?}, w_3: {:?} [?/s]",
                omega_rot[0], omega_rot[1], omega_rot[2]
            );
        }

        if recenter {
            fwd_kinematics.re_center_ref();
            recenter = false; // this will later depend on keyboard input
        }

        let dt = 1.0 / bravo_twist.sample_rate;

        let next_pos: [f64; 3] = [
            dt * xyz_vel[0] + bravo_twist.pose[0],
            dt * xyz_vel[1] + bravo_twist.pose[1],
            dt * xyz_vel[2] + bravo_twist.pose[2],
        ];

        let next_euler = integrate_first_order(&bravo_twist.quat, &omega_rot, dt);

        if cli.debug {
            println!("[Bravo7] New target pose:");
            if let [w, x, y, z] = &next_euler[..] {
                println!("w: {:?}, x: {:?}, y: {:?}, z: {:?}", w, x, y, z);
            }
            if let [x, y, z] = &next_pos[..] {
                println!("x: {:?}, y: {:?}, z: {:?}", x, y, z);
            }
        }

        responder.send("TEST", 0).unwrap();
    }
}
