use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use clap::Parser;
use serde::Deserialize;

use feetech_bravo_teleop::{Driver, ReadCommand::CurrentPosition, So100FwdKinematics};

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
}

#[derive(Debug, Deserialize)]
struct JointCalibration {
    id: u8,
    drive_mode: u8,
    homing_offset: i32,
    range_min: i32,
    range_max: i32,
}

#[derive(Debug, Deserialize)]
struct JointState {
    calibration: JointCalibration,
    #[serde(default)]
    current_step: u16,
    #[serde(default)]
    current_rads: f32,
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

    let mut servo_states: HashMap<u8, JointState> = servo_calib
        .into_values()
        .map(|calib| {
            (
                calib.id,
                JointState {
                    calibration: calib,
                    current_step: 0,
                    current_rads: 0.0,
                },
            )
        })
        .collect();
    if cli.debug {
        println!("Servo states:\n{:?}", servo_states);
    }

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_clone.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    let mut servo_positions: Vec<u16> = [0; 6].to_vec();
    let mut teleop_input = Driver::new(&cli.port);

    let mut recenter = true;
    let mut fwd_kinematics = So100FwdKinematics::new();
    while running.load(std::sync::atomic::Ordering::SeqCst) {
        for motor_id in 1u8..=6u8 {
            servo_positions[(motor_id - 1) as usize] =
                teleop_input.read(motor_id, CurrentPosition).unwrap();
            if let Some(servo_info) = servo_states.get_mut(&motor_id) {
                servo_info.current_step = servo_positions[(motor_id - 1) as usize];
                servo_info.current_rads = step_to_rads(
                    servo_info.current_step as i32,
                    servo_info.calibration.homing_offset,
                );
                fwd_kinematics
                    .update_theta((motor_id - 1) as usize, servo_info.current_rads);
            }
        }
        fwd_kinematics.update_pose_twist();

        if recenter {
            fwd_kinematics.re_center_ref();
            recenter = false; // this will later depend on keyboard input
        }

        if cli.debug {
            println!("Current Servo Angles (rads):");
            for (servo_id, joint_info) in &servo_states {
                println!("{}: {:.4}", servo_id, joint_info.current_rads);
            }
            let ee_pos = fwd_kinematics.get_ee_position();
            println!("Current end effector position:");
            println!("x: {}, y: {}, z: {}", ee_pos[0], ee_pos[1], ee_pos[2]);
            let ee_quat = fwd_kinematics.get_ee_rotation();
            println!("Current end effector rotation (quaternion):");
            println!("w: {}, x: {}, y: {}, z: {}", ee_quat[0], ee_quat[1], ee_quat[2], ee_quat[3]);
        }
    }
}
