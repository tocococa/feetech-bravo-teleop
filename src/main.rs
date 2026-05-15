use std::fs::File;
// use std::io::{BufRead, Write};
// use std::sync::atomic::AtomicBool;
// use std::sync::Arc;
// use std::{io, thread::sleep, time::Duration};
use std::io::{BufRead, self};

mod driver;
mod commands;
mod packet_handler;

fn read_calibration_file(name: &str) -> Vec<i32> {
    let file = File::open(format!("./calibration/{}", name)).expect("File not found");
    let mut reader = io::BufReader::new(file);
    let mut values = String::new();
    reader.read_line(&mut values).unwrap();
    values
        .split(",")
        .map(|v| v.trim().parse::<i32>().unwrap())
        .collect()
}

fn main() {
    println!("Hello, world!");
}
