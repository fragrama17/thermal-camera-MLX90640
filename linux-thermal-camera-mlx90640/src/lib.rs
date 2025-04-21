use std::io;
use std::io::Error;
use std::thread::sleep;
use std::time::Duration;
use i2cdev::core::{I2CDevice, I2CTransfer};
use i2cdev::linux::{I2CMessage, LinuxI2CDevice, LinuxI2CMessage};

pub struct ThermalCamera {
    address: u16,
    bus_id: i32,
    device: LinuxI2CDevice,
    params_mlx: ParamsMlx
}

/**
 * The programmable refresh rate of the thermal camera (2Hz -> 1fps, 4Hz -> 2fps ecc).
 *
 * Note that the sensor needs to populate 2 sub-pages of the matrix, therefore delay is doubled
 */
#[derive(Debug, Clone, Copy)]
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
    /**
     * Available frame every 0.25s
     */
    _8Hz = 0b100,
    /**
     * Available frame every 0.125s
     */
    _16Hz = 0b101,
    /**
     * Available frame every 0.125s
     */
    _32Hz = 0b110,
    /**
     * Available frame every 0.0625s
     */
    _64Hz = 0b111,
}

impl TryFrom<u8> for RefreshRate {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(RefreshRate::_0_5Hz),
            1 => Ok(RefreshRate::_1Hz),
            2 => Ok(RefreshRate::_2Hz),
            3 => Ok(RefreshRate::_4Hz),
            4 => Ok(RefreshRate::_8Hz),
            5 => Ok(RefreshRate::_16Hz),
            6 => Ok(RefreshRate::_32Hz),
            7 => Ok(RefreshRate::_64Hz),
            _ => Err(()),
        }
    }
}

const STATUS_REGISTER: u16 = 0x8000;
const CONTROL_REGISTER: u16 = 0x800D;
const CONFIGURATION_REGISTER: u16 = 0x800F;
const RAM_START_REGISTER: u16 = 0x0400;
const RAM_END_REGISTER: u16 = 0x06FF;
const AUX_DATA_START_ADDRESS: u16 = 0x0700;
const EE_PROM_START_ADDRESS: u16 = 0x2400;

const FRAME_SIZE: usize = 834;
const TOT_PIXELS: usize = 768;
const TOT_COLUMNS: usize = 32;
const TOT_ROWS: usize = 24;

const FRAME_DATA_ERROR: i32 = -8;


impl ThermalCamera {
    pub fn new(address: u16, bus_id: i32) -> Self {
        Self {
            address,
            bus_id,
            device: LinuxI2CDevice::new(format!("/dev/i2c-{}", bus_id), address).unwrap(),
            params_mlx: ParamsMlx::default()
        }
    }

    pub fn init_parameters(&mut self) {
        // TODO read from eeprom registers to init constants for To calculation
        let mut eeprom_data = [0u16; 832];
        self.read_words_from_register(EE_PROM_START_ADDRESS, &mut eeprom_data);
        self.params_mlx.extract_vdd_parameters(&eeprom_data);
        self.params_mlx.extract_ptat_parameters(&eeprom_data);
        self.params_mlx.extract_gain_parameters(&eeprom_data);
        self.params_mlx.extract_tgc_parameters(&eeprom_data);
        self.params_mlx.extract_resolution_parameters(&eeprom_data);
        self.params_mlx.extract_ks_ta_parameters(&eeprom_data);
    }

    pub fn get_image(&mut self) -> Result<[f32; TOT_PIXELS], Error> {
        let emissivity = 0.95;
        let mut frame_data = [0u16; FRAME_SIZE];
        let mut frame = [0.0f32; TOT_PIXELS];

        for i in 0..2 { // first sub-page 0, then sub-page 1
            println!("getting sub-page {}", i);
            // Fetch the frame data asynchronously
            let status = self.get_frame_data(&mut frame_data);

            if status < 0 {
                return Err(Error::new(io::ErrorKind::Other, "error while getting data frame"));
            }

            // let tr = get_ta(&frame_data) - 8.0;

            // Calculate To for pixels
            // calculate_to(&frame_data, emissivity, tr, &mut frame);
        }

        if frame_data.iter().take(TOT_PIXELS).any(|&w| w == 0) {
            println!("Failed to populate both sub-pages");
        }

        for i in 0..TOT_PIXELS {
            frame[i] = frame_data[i] as f32;
        }

        Ok(frame)
    }

    fn get_frame_data(&mut self, frame_data: &mut [u16; FRAME_SIZE]) -> i32 {
        let mut status;
        let mut data_ready: u16 = 0;
        let mut status_word: u16 = 0;

        while data_ready == 0
        {
            status_word = self.read_word_from_register(STATUS_REGISTER);

            data_ready = (status_word >> 3) & 0b1;
        }

        while data_ready != 0 {
            status = self.write_init_value_to_status_register();

            if status < 0 {
                return status;
            }

            self.read_words_from_register(RAM_START_REGISTER, frame_data);

            status_word = self.read_word_from_register(STATUS_REGISTER);

            data_ready = (status_word >> 3) & 0b1;
        }

        let control_word = self.read_word_from_register(CONTROL_REGISTER);

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

    pub fn get_refresh_rate(&mut self) -> RefreshRate {
        let word = self.read_word_from_register(CONTROL_REGISTER);

        RefreshRate::try_from((word >> 7 & 0b111) as u8).unwrap()
    }

    pub fn set_refresh_rate(&mut self, refresh_rate: RefreshRate) {
        let control_word = self.read_word_from_register(CONTROL_REGISTER);
        let new_refresh_rate = (refresh_rate as u16) << 7;
        let new_control_word = new_refresh_rate | (control_word & 0xFC7F);  // 0b1111110001111111, the 3 bit set to 0 to merge | with new refresh-rate

        self.write_word_to_register(CONTROL_REGISTER, new_control_word);
    }

    fn validate_frame_data(frame_data: &[u16; 834]) -> i32 {
        let mut line = 0;

        for i in (0..TOT_PIXELS).step_by(TOT_COLUMNS)
        {
            if frame_data[i as usize] == 0x7FFF && line % 2 == frame_data[833] {
                return FRAME_DATA_ERROR;
            }

            line += 1;
        }

        return 0;
    }

    fn read_words_from_register(&mut self, register: u16, words: &mut [u16])
    {
        let mut words_buffer = vec![0; &words.len() * 2]; // Create a dynamically sized buffer

        self.read_from_register(register, &mut words_buffer);

        for i in (0..words_buffer.len()).step_by(2) {
            words[i / 2] = ((words_buffer[i] as u16) << 8) | (words_buffer[i + 1] as u16); // MSB at index 0, LSB at index 1
        }
    }

    fn write_init_value_to_status_register(&mut self) -> i32 {
        let init_word: u16 = 0x0030;
        let sub_page0check = 0x10;
        let sub_page1check = 0x11;

        // TODO test if transfer method can fix slow frame-rate limit

        self.write_word_to_register(STATUS_REGISTER, init_word);

        sleep(Duration::from_millis(1));

        let data_check = self.read_word_from_register(STATUS_REGISTER);
        if data_check == sub_page0check || data_check == sub_page1check {
            return data_check as i32;
        }

        return -2;
    }

    fn write_word_to_register(&mut self, register: u16, word: u16) {
        let mut cmd: [u8; 4] = [0; 4];

        cmd[0] = (register >> 8) as u8;
        cmd[1] = (register & 0xFF) as u8;
        cmd[2] = (word >> 8) as u8;
        cmd[3] = (word & 0xFF) as u8;

        let _ = self.device.write(&mut cmd);
    }

    fn read_word_from_register(&mut self, register: u16) -> u16 {
        let mut word_buffer: [u8; 2] = [0; 2];

        self.read_from_register(register, &mut word_buffer);

        ((word_buffer[0] as u16) << 8) | (word_buffer[1] as u16)    // MSB at index 0, LSB at index 1
    }

    fn read_from_register(&mut self, register: u16, mut read_buffer: &mut [u8]) {
        let mut register_buffer: [u8; 2] = [0; 2];

        register_buffer[0] = (register >> 8) as u8;
        register_buffer[1] = (register & 0xFF) as u8;

        let _ = self.device.transfer(&mut [
            LinuxI2CMessage::write(&mut register_buffer).with_address(self.address),
            LinuxI2CMessage::read(&mut read_buffer).with_address(self.address),
        ]);
    }
}

struct ParamsMlx {
    pub k_vdd: i16,
    pub vdd25: i16,
    pub kv_ptat: f32,
    pub kt_ptat: f32,
    pub vp_tat25: u16,
    pub alpha_ptat: f32,
    pub gain_ee: i16,
    pub tgc: f32,
    pub cp_kv: f32,
    pub cp_kta: f32,
    pub resolution_ee: u8,
    pub calibration_mode_ee: u8,
    pub ks_ta: f32,
    pub ks_to: [f32; 5],
    pub ct: [i16; 5],
    pub alpha: [u16; 768],
    pub alpha_scale: u8,
    pub offset: [i16; 768],
    pub kta: [i8; 768],
    pub kta_scale: u8,
    pub kv: [i8; 768],
    pub kv_scale: u8,
    pub cp_alpha: [f32; 2],
    pub cp_offset: [i16; 2],
    pub il_chess_c: [f32; 3],
    pub broken_pixels: [u16; 5],
    pub outlier_pixels: [u16; 5],
}

impl ParamsMlx {
    fn extract_vdd_parameters(&mut self, eeprom_data: &[u16]) {
        let k_vdd = ((eeprom_data[51] & 0xFF00) >> 8) as i8;
        let mut vdd25 = (eeprom_data[51] & 0x00FF) as i16;
        vdd25 = ((vdd25 - 256) << 5) - 8192;

        self.k_vdd = 32 * k_vdd as i16;
        self.vdd25 = vdd25;
    }

    fn extract_ptat_parameters(&mut self, ee_data: &[u16]) {
        let mut kv_ptat = ((ee_data[50] & 0xFC00) >> 10) as f32;
        if kv_ptat > 31.0 {
            kv_ptat -= 64.0;
        }
        kv_ptat /= 4096.0;

        let mut kt_ptat = (ee_data[50] & 0x03FF) as f32;
        if kt_ptat > 511.0 {
            kt_ptat -= 1024.0;
        }
        kt_ptat /= 8.0;

        let v_ptat25 = ee_data[49];
        let alpha_ptat = ((ee_data[16] & 0xF000) as f32 / 2f32.powf(14.0)) + 8.0;

        self.kv_ptat = kv_ptat;
        self.kt_ptat = kt_ptat;
        self.vp_tat25 = v_ptat25;
        self.alpha_ptat = alpha_ptat;
    }

    fn extract_gain_parameters(&mut self, ee_data: &[u16]) {
        self.gain_ee = ee_data[48] as i16;
    }

    fn extract_tgc_parameters(&mut self, ee_data: &[u16]) {
        // what a hell was this
        // self.tgc = (ee_data[60] as i8 & 0x00FF) as f32 / 32.0;
        self.tgc = (ee_data[60] as u8 & 0xFF) as f32 / 32.0;
    }

    fn extract_resolution_parameters(&mut self, ee_data: &[u16]) {
        self.resolution_ee = ((ee_data[56] & 0x3000) >> 12) as u8;
    }

    fn extract_ks_ta_parameters(&mut self, ee_data: &[u16]) {
        self.ks_ta = (((ee_data[60] & 0xFF00) >> 8) as i8) as f32 / 8192.0;
    }
}

impl Default for ParamsMlx {
    fn default() -> Self {
        Self {
            k_vdd: 0,
            vdd25: 0,
            kv_ptat: 0.0,
            kt_ptat: 0.0,
            vp_tat25: 0,
            alpha_ptat: 0.0,
            gain_ee: 0,
            tgc: 0.0,
            cp_kv: 0.0,
            cp_kta: 0.0,
            resolution_ee: 0,
            calibration_mode_ee: 0,
            ks_ta: 0.0,
            ks_to: [0.0; 5],
            ct: [0; 5],
            alpha: [0; 768],
            alpha_scale: 10,
            offset: [0; 768],
            kta: [0; 768],
            kta_scale: 0,
            kv: [0; 768],
            kv_scale: 7,
            cp_alpha: [0.0; 2],
            cp_offset: [0; 2],
            il_chess_c: [0.0; 3],
            broken_pixels: [0; 5],
            outlier_pixels: [0; 5],
        }
    }
}
