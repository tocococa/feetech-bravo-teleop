// use std::fs::File;
// use std::io::{self, BufRead};

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use clap::Parser;
use serde::Deserialize;
// use std::io::{BufRead, Write};
// use std::sync::atomic::AtomicBool;
// use std::sync::Arc;
// use std::{io, thread::sleep, time::Duration};

mod driver;
// mod commands;
// mod packet_handler;

#[derive(Parser, Debug)]
#[command(
    version, about, long_about = None,
    after_help = "
    Keep terminal in focus to use, press space to engage teleop mode, relese to disengage.\n
    Press space and C together to engage the gripper control.\n
    Remember to allow control of the serial port at which the Feetech connected to (e.g., sudo chmod 666 /dev/ttyACM0)"
)]
struct Cli {
    #[arg(short, long, default_value = "./calibration.json")]
    calibration_file: String,
    #[arg(short, long, default_value = "ttyACM0")]
    port: String,
}

#[derive(Debug, Deserialize)]
struct JointConfig {
    id: u8,
    drive_mode: u8,
    homing_offset: i32,
    range_min: i32,
    range_max: i32,
}

fn main() {
    let cli = Cli::parse();
    let json =
        std::fs::read_to_string(cli.calibration_file).expect("Failed to read calibration file");
    let config: HashMap<String, JointConfig> =
        serde_json::from_str(&json).expect("Failed to parse calibration file");

    for (joint_name, joint_config) in &config {
        println!("Joint: {}, Config: {:?}", joint_name, joint_config);
    }

    let running = Arc::new(AtomicBool::new(true));
    let running_clone = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_clone.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    let mut servo_positions: Vec<u16> = [0; 6].to_vec();
    let mut teleop = Driver::new(&cli.port);

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        for motor_id in 1u8..=6u8 {
            servo_positions[(motor_id - 1) as usize] = 0; // TODO: read from servo
        }
    }
}
