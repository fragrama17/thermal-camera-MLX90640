use i2cdev::core::{I2CDevice, I2CTransfer};
use i2cdev::linux::{I2CMessage, LinuxI2CDevice, LinuxI2CMessage};

pub fn write_word_to_register(device: &mut LinuxI2CDevice, register: u16, word: u16) {
    let mut cmd: [u8; 4] = [0; 4];

    cmd[0] = (register >> 8) as u8;
    cmd[1] = (register & 0xFF) as u8;
    cmd[2] = (word >> 8) as u8;
    cmd[3] = (word & 0xFF) as u8;

    let _ = device.write(&mut cmd);
}

pub fn read_word_from_register(device: &mut LinuxI2CDevice, address: u16, register: u16) -> u16 {
    let mut word_buffer: [u8; 2] = [0; 2];

    read_from_register(device, address, register, &mut word_buffer);

    ((word_buffer[0] as u16) << 8) | (word_buffer[1] as u16)    // MSB at index 0, LSB at index 1
}

fn read_from_register(device: &mut LinuxI2CDevice, address: u16, register: u16, mut read_buffer: &mut [u8]) {
    let mut register_buffer: [u8; 2] = [0; 2];

    register_buffer[0] = (register >> 8) as u8;
    register_buffer[1] = (register & 0xFF) as u8;

    let _ = device.transfer(&mut [
        LinuxI2CMessage::write(&mut register_buffer).with_address(address),
        LinuxI2CMessage::read(&mut read_buffer).with_address(address),
    ]);
}

pub fn read_words_from_register(device: &mut LinuxI2CDevice, address: u16, register: u16, words: &mut [u16])
{
    let mut words_buffer = vec![0; &words.len() * 2]; // Create a dynamically sized buffer

    read_from_register(device, address, register, &mut words_buffer);

    for i in (0..words_buffer.len()).step_by(2) {
        words[i / 2] = ((words_buffer[i] as u16) << 8) | (words_buffer[i + 1] as u16); // MSB at index 0, LSB at index 1
    }
}