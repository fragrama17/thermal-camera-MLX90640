use core::marker::PhantomData;
use embedded_hal::i2c::{I2c, Operation, SevenBitAddress};

pub struct I2cUtils<T: I2c> {
    phantom_data: PhantomData<T>    
}

impl <T: I2c> I2cUtils<T> {
    pub fn write_word_to_register(device: &mut T, address: SevenBitAddress, register: u16, word: u16) -> Result<(), <T>::Error> {
        let mut cmd: [u8; 4] = [0; 4];

        cmd[0] = (register >> 8) as u8;
        cmd[1] = (register & 0xFF) as u8;
        cmd[2] = (word >> 8) as u8;
        cmd[3] = (word & 0xFF) as u8;

        device.write(address, &mut cmd)
    }

    pub fn read_word_from_register(device: &mut T, address: SevenBitAddress, register: u16) -> Result<u16, <T>::Error> {
        let mut word_buffer: [u8; 2] = [0; 2];

        Self::read_from_register(device, address, register, &mut word_buffer)?;

        Ok(((word_buffer[0] as u16) << 8) | (word_buffer[1] as u16))    // MSB at index 0, LSB at index 1
    }

    fn read_from_register(device: &mut T, address: SevenBitAddress, register: u16, mut read_buffer: &mut [u8]) -> Result<(), <T>::Error> {
        let mut register_buffer: [u8; 2] = [0; 2];

        register_buffer[0] = (register >> 8) as u8;
        register_buffer[1] = (register & 0xFF) as u8;

        device.transaction(address, &mut [
            Operation::Write(&register_buffer),
            Operation::Read(&mut read_buffer),
        ])
    }

    pub fn read_words_from_register<const N: usize>(device: &mut T, address: SevenBitAddress, register: u16, words: &mut [u16]) -> Result<(), <T>::Error>
    {
        let mut words_buffer: heapless::Vec<u8, N> = heapless::Vec::new();
        // without resizing vector will result in a 0-sized vector at compile time
        words_buffer.resize_default(N).unwrap();

        Self::read_from_register(device, address, register, words_buffer.as_mut_slice())?;

        for i in (0..words_buffer.len()).step_by(2) {
            words[i / 2] = ((words_buffer[i] as u16) << 8) | (words_buffer[i + 1] as u16); // MSB at index 0, LSB at index 1
        }

        Ok(())
    }    
}