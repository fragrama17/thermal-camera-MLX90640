use defmt::*;
use embedded_hal_async::i2c;

pub async fn write_word_to_register<I2c: i2c::I2c>(device: &mut I2c, address: u8, register: u16, word: u16)
{
    let mut cmd: [u8; 4] = [0; 4];

    cmd[0] = (register >> 8) as u8;
    cmd[1] = (register & 0xFF) as u8;
    cmd[2] = (word >> 8) as u8;
    cmd[3] = (word & 0xFF) as u8;

    debug!("writing word to register {}", register);

    let r = device.write(address, &mut cmd).await;

    match r {
        Ok(_) => {
            debug!("word written successfully");
        }
        Err(_) => {
            error!("failed to write word to register {}", register)
        }
    }
}

pub async fn read_word_from_register<I2c: i2c::I2c>(device: &mut I2c, address: u8, register: u16) -> u16 {
    let mut word_buffer: [u8; 2] = [0; 2];

    read_from_register(device, address, register, &mut word_buffer).await;

    ((word_buffer[0] as u16) << 8) | (word_buffer[1] as u16)    // MSB at index 0, LSB at index 1
}

pub async fn read_from_register<I2c: i2c::I2c>(device: &mut I2c, address: u8, register: u16, mut read_buffer: &mut [u8])
{
    let mut register_buffer: [u8; 2] = [0; 2];

    register_buffer[0] = (register >> 8) as u8;
    register_buffer[1] = (register & 0xFF) as u8;

    debug!("reading from register {}", register);

    let r = device.write_read(address, &register_buffer, &mut read_buffer).await;

    match r {
        Ok(_) => {
            debug!("bytes read: {}", read_buffer.len());
        }
        Err(_) => {
            error!("failed to read {} bytes from register {}", read_buffer.len(), register);
        }
    }
}

pub async fn read_words_from_register<I2c: i2c::I2c, const N: usize>(device: &mut I2c, address: u8, register: u16, words: &mut [u16])
{
    let mut words_buffer: heapless::Vec<u8, N> = heapless::Vec::new();
    words_buffer.resize_default(N).unwrap();
    
    debug!("allocated Vec on the stack of size {}", words_buffer.len());

    read_from_register(device, address, register, &mut words_buffer).await;

    for i in (0..words_buffer.len()).step_by(2) {
        words[i / 2] = ((words_buffer[i] as u16) << 8) | (words_buffer[i + 1] as u16); // MSB at index 0, LSB at index 1

        // debug!("assigning word {} at index {}", words[i / 2], i);
    }
}