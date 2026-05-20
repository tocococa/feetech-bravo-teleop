pub fn compute_checksum(id: u8, length: u8, instruction: u8, parameters: &[u8]) -> u8 {
    // https://emanual.robotis.com/docs/en/dxl/protocol1/#checksum-instruction-packet
    let mut checksum: u16 = 0; // avoid overflows, so set as u16
    checksum += id as u16;
    checksum += length as u16;
    checksum += instruction as u16;
    for param in parameters {
        checksum += *param as u16;
    }
    (!checksum & 0xff) as u8
}

const STEPS_PER_REV: i32 = 4096;
const RADS_PER_STEP: f32 = 2.0 * std::f32::consts::PI / STEPS_PER_REV as f32;

pub fn step_to_rads(current_step: i32, zero_step: i32) -> f32 {
    let mut delta = (current_step - zero_step) % STEPS_PER_REV as i32;
    if delta < 0 {
        delta += STEPS_PER_REV;
    }
    let mut angle = delta as f32 * RADS_PER_STEP;
    if angle > std::f32::consts::PI / 2.0 {
        angle -= 2.0 * std::f32::consts::PI;
    }
    angle
}