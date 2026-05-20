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

The low-level serial driver for the Feetech board is based on the [feetech-servo-rs](https://github.com/proteusvacuum/feetech-servo-rs/tree/main) library by the Recurse Center. It has had some of its functionality sifted, as this module only needs to read the state of the SO-100 and then map that radians.

### Extras

Bravo 7 Denavit-Hartenberg params:
```yaml
theta0: 
  d: 0.1074
  theta: theta0 + pi
  a: 0.0460
  alpha: +pi/2
  
theta1: 
  d: 0
  theta: theta1 - pi/2 + theta_a
  a: 0.2936
  alpha: 0
  
theta2: 
  d: 0
  theta: theta2 - pi/2 - theta_a
  a: 0.0408
  alpha: -pi/2
  
theta3: 
  d: -0.1600
  theta: theta3
  a: 0.0408
  alpha: -pi/2
  
theta4: 
  d: 0
  theta: theta4
  a: 0.0408
  alpha: -pi/2
  
theta5: 
  d: -0.2235
  theta: theta5
  a: 0
  alpha: +pi/2
  
end_effector:
  d: 0
  theta: -pi/2 (fixed)
  a: 0.1200
  alpha: 0
```

SO-100 classic Denavit-Hartenberg params (classic DH):
```yaml
# Format (classic DH): for link i
# theta{i}:
#   d: d_i (m)
#   theta: theta_i (+ offset)
#   a: a_i (m)
#   alpha: alpha_i (rad)

theta0:
  d: 0.0165
  theta: theta0
  a: 0.0306
  alpha: 1.5708

theta1:
  d: 0.1025
  theta: theta1 - 1.8
  a: 0.1160
  alpha: 0.0

theta2:
  d: 0.0
  theta: theta2 + 1.571
  a: 0.1350
  alpha: 0.0

theta3:
  d: 0.0
  theta: theta3 - 1.0
  a: 0.0
  alpha: 1.5708

theta4:
  d: 0.0
  theta: theta4 + 1.571
  a: 0.0202
  alpha: 1.5708

end_effector:
  d: 0.0244
  theta: -pi/2 (fixed)
  a: 0.0
  alpha: 0.0
```

Notes:
- These classic DH parameters were derived from the joint origin and axis data you supplied by computing the common normals and twists between successive joint axes (r_i, alpha_i) and the offsets (theta shifts) needed so that theta_i=0 matches the provided RPY orientation.
- Units: distances in meters, angles in radians.
