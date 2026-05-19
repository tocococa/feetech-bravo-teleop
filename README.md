# Bravo teleop

This repo contains a Rust driver for a SO-100 arm in leader configuration with Feetech STS3215 servos to be used as a teleoperation interface for a Reach Robotics Bravo 7 arm. The driver is designed to be used with ROS2 and sends joint velocity commands to the Bravo 7 arm driver.

## Usage

Run

## Build and details

Build with `cargo build --release` and run with `./target/release/feetech-bravo-teleop -p /dev/ttyACM0 -c calibration.xml`, replacing the port and calibration file as needed.

The driver will publish velocity commands as `/bravo_7_teleop/joint_velocity_command` and subscribe to joint states from the Bravo 7 arm on `/bravo_7/joint_states`. The driver will also publish the current joint states of the SO-100 arm on `/so_100/joint_states` for visualization in RViz or other tools.

## Prerequisites

- Rust toolchain (stable): `$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
- ROS2 (Humble or later): Follow the official ROS2 installation guide for your platform: https://docs.ros.org/en/humble/Installation.html
- libudev (for Rust serial library): `$ sudo apt update && sudo apt install libudev-dev pkg-config`

## References

The low-level serial driver for the Feetech board is based on the [feetech-servo-rs](https://github.com/proteusvacuum/feetech-servo-rs/tree/main) library by the Recurse Center. It has had most of its functionality sifted, as this module only needs to read the state of the SO-100.
