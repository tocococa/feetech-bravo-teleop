use super::memory_location::{
    ACCELERATION, CURRENT_POSITION, ID, TEMPERATURE,
};

use super::packets::InstructionPacket;

pub trait IntoInstructionPacket {
    fn to_instruction_packet(&self, motor_id: u8) -> InstructionPacket;
}

pub enum ReadCommand {
    Acceleration,
    Id,
    CurrentPosition,
    Temperature,
}

impl IntoInstructionPacket for ReadCommand {
    fn to_instruction_packet(&self, motor_id: u8) -> InstructionPacket {
        match self {
            ReadCommand::Id => InstructionPacket::read_from_memory_location(motor_id, ID),
            ReadCommand::CurrentPosition => {
                InstructionPacket::read_from_memory_location(motor_id, CURRENT_POSITION)
            }
            ReadCommand::Temperature => {
                InstructionPacket::read_from_memory_location(motor_id, TEMPERATURE)
            }
            ReadCommand::Acceleration => {
                InstructionPacket::read_from_memory_location(motor_id, ACCELERATION)
            }
        }
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use crate::instruction::Instruction;

//     #[test]
//     fn command_instruction_packet() {
//         let motor_id = 0x25;

//         assert_eq!(
//             ReadCommand::Temperature.to_instruction_packet(motor_id),
//             InstructionPacket::new(motor_id, Instruction::Read, &[0x3F, 1])
//         );
//     }
// }
