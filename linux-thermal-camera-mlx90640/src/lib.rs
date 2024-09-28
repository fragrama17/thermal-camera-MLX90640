use std::os::linux::raw::stat;
use std::sync::atomic::AtomicI32;
use i2cdev::core::*;
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;

// pub fn add(left: u64, right: u64) -> u64 {
//     left + right
// }
//
// #[cfg(test)]
// mod tests {
//     use super::*;
//
//     #[test]
//     fn it_works() {
//         let result = add(2, 2);
//         assert_eq!(result, 4);
//     }
// }

pub struct ThermalCamera {
    address: u16,
    bus_id: i32,
    device: LinuxI2CDevice,
}

const STATUS_REGISTER: u16 = 0x8000;
const CONTROL_REGISTER: u16 = 0x800D;
const CONFIGURATION_REGISTER: u16 = 0x800F;
const RAM_START_REGISTER: u16 = 0x0400;
const RAM_END_REGISTER: u16 = 0x06FF;
const AUX_DATA_START_ADDRESS: u16 = 0x0700;
const EE_PROM_START_ADDRESS: u16 = 0x2400;

const TOT_PIXELS: i32 = 768;
const TOT_COLUMNS: usize = 32;
const TOT_ROWS: usize = 24;

const FRAME_DATA_ERROR: i32 = -8;


impl ThermalCamera {
    pub fn new(address: u16, bus_id: i32) -> Self {
        Self {
            address,
            bus_id,
            device: LinuxI2CDevice::new(format!("/dev/i2c-{}", bus_id), address).unwrap(),
        }
    }

    pub fn get_image() -> [f32; 768] {
        [0.0; 768]
    }

    fn get_frame_data(frame_data: &mut [u16; 834]) -> i32 {
        let mut status = 0;
        let mut data_ready: u16 = 0;
        let mut status_word: u16 = 0;

        while data_ready == 0
        {
            status_word = Self::read_word_from_register(STATUS_REGISTER);

            data_ready = (status_word >> 3) & 0b1;
        }

        while data_ready != 0 {
            status = Self::write_init_value_to_status_register();

            if status < 0 {
                return status;
            }

            Self::read_words_from_register(RAM_START_REGISTER, frame_data);

            status_word = Self::read_word_from_register(STATUS_REGISTER);

            data_ready = (status_word >> 3) & 0b1;
        }

        let control_word = Self::read_word_from_register(CONTROL_REGISTER);

        frame_data[832] = control_word;
        frame_data[833] = status_word & 0x0001;

        status = Self::validate_frame_data(frame_data);
        if status != 0
        {
            return status;
        }

        // status = ValidateAuxData(frameData.Skip(TotPixels).Take(TotAuxData).ToArray());
        // if (status != 0)
        // {
        //     // Console.WriteLine("aux data validation failed");
        //     return frameData[833];
        // }

        return frame_data[833] as i32;
    }

    fn validate_frame_data(frame_data: &[u16; 834]) -> i32 {
        let mut line = 0;

        for i in (0..TOT_PIXELS).step_by(TOT_COLUMNS)
        {
            if frame_data[i] == 0x7FFF && line % 2 == frame_data[833] {
                return FRAME_DATA_ERROR;
            }

            line += 1;
        }

        return 0;
    }

    fn read_words_from_register<const N: usize>(register: u16, words: &mut [u16; N])
    {
        let mut words_buffer = [0; N * 2];

        Self::read_from_register(register, words_buffer);

        for i in (0..words_buffer.len()).step_by(2) {
            words[i / 2] = ((words_buffer[i] << 8) | words_buffer[i + 1]) as u16; // MSB at index 0, LSB at index 1
        }
    }

    fn write_init_value_to_status_register() -> i32 {
        let init_word: u16 = 0x0030;
        let sub_page0check = 0x10;
        let sub_page1check = 0x11;

        Self::write_word_to_register(STATUS_REGISTER, init_word);

        let data_check = Self::read_word_from_register(STATUS_REGISTER);
        if data_check == sub_page0check || data_check == sub_page1check {
            return data_check as i32;
        }

        return -2;
    }

    fn write_word_to_register(register: u16, word: u16) {
        let mut cmd: [u8; 4] = [0; 4];

        cmd[0] = (register >> 8) as u8;
        cmd[1] = (register & 0xFF) as u8;
        cmd[2] = (word >> 8) as u8;
        cmd[3] = (word & 0xFF) as u8;

        let _ = Self.device.write(cmd.borrow());
    }

    fn read_word_from_register(register: u16) -> u16 {
        let mut word_buffer: [u8; 2] = [0; 2];

        Self::read_from_register(register, word_buffer);

        return ((word_buffer[0] << 8) | word_buffer[1]) as u16; // MSB at index 0, LSB at index 1
    }

    fn read_from_register<const N: usize>(register: u16, mut read_buffer: [u8; N]) {
        let mut register_buffer: [u8; 2] = [0; 2];

        register_buffer[0] = (register >> 8) as u8;
        register_buffer[1] = (register & 0xFF) as u8;

        let _ = Self.device.write(&mut register_buffer);

        let _ = Self.device.read(&mut read_buffer);
    }
}

/**
 * The programmable refresh rate of the thermal camera (2Hz -> 1fps, 4Hz -> 2fps ecc).
 *
 * Note that the sensor needs to populate 2 sub-pages of the matrix, therefore delay is doubled
 */
pub enum RefreshRate {
    /**
     * Available frame every 4s
     */
    _0_5Hz = 0b000,
    /**
     * Available frame every 2s
     */
    _1Hz = 0b001,
    /**
     * Available frame every 1s
     */
    _2Hz = 0b010,
    /**
     * Available frame every 0.5s
     */
    _4Hz = 0b011,
}

