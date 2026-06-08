#![allow(dead_code)]
mod feetech_driver;
pub use feetech_driver::Driver;
pub use feetech_driver::commands::ReadCommand;
pub use feetech_driver::utils;

mod kinematics;
pub use kinematics::forward::So100FwdKinematics;
pub use kinematics::utils::Twist;
pub use kinematics::utils::integrate_first_order;
