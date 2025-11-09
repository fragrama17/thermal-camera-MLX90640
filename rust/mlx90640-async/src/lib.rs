use std::io;
use std::io::Error;
use crate::i2c_utils::{read_word_from_register, read_words_from_register, write_word_to_register};

mod i2c_utils;

const STATUS_REGISTER: u16 = 0x8000;
const CONTROL_REGISTER: u16 = 0x800D;
const CONFIGURATION_REGISTER: u16 = 0x800F;
const RAM_START_REGISTER: u16 = 0x0400;
const RAM_END_REGISTER: u16 = 0x06FF;
const AUX_DATA_START_ADDRESS: u16 = 0x0700;
const EE_PROM_START_ADDRESS: u16 = 0x2400;

const FRAME_SIZE: usize = 834;
pub const TOT_PIXELS: usize = 768;
pub const TOT_COLUMNS: usize = 32;
pub const TOT_ROWS: usize = 24;
const SCALE_ALPHA: f32 = 0.000001;

const FRAME_DATA_ERROR: i32 = -8;

pub struct ThermalCamera<I2c>
where
    I2c: embedded_hal_async::i2c::I2c,
{
    address: u8,
    device: I2c,
    params_mlx: ParamsMlx,
}

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
     * Available frame every 0.0625s
     */
    _32Hz = 0b110,
    /**
     * Available frame every 0.03125s
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

impl<I2c> ThermalCamera<I2c>
where
    I2c: embedded_hal_async::i2c::I2c,
{
    pub fn new(address: u8, mut device: I2c) -> Self
    where
        I2c: embedded_hal_async::i2c::I2c,
    {
        let mut params_mlx = ParamsMlx::default();
        params_mlx.init_parameters(&mut device, address);

        Self {
            address,
            device,
            params_mlx,
        }
    }

    pub async fn get_image(&mut self) -> Result<[f32; TOT_PIXELS], Error> {
        let emissivity = 0.95;
        let mut frame_data = [0u16; FRAME_SIZE];
        let mut frame = [0.0f32; TOT_PIXELS];

        for _i in 0..2 {
            let status = self.get_frame_data(&mut frame_data).await;

            if status < 0 {
                return Err(Error::new(io::ErrorKind::Other, "error while getting data frame"));
            }

            let tr = self.get_ta(&frame_data) - 8.0;

            // Calculate To for pixels
            self.calculate_to(&frame_data, emissivity, tr, &mut frame);
        }

        Ok(frame)
    }

    async fn get_frame_data(&mut self, frame_data: &mut [u16; FRAME_SIZE]) -> i32 {
        let mut status;
        let mut data_ready: u16 = 0;
        let mut status_word: u16 = 0;

        while data_ready == 0
        {
            status_word = read_word_from_register(&mut self.device, self.address, STATUS_REGISTER).await;

            data_ready = (status_word >> 3) & 0b1;
        }

        while data_ready != 0 {
            status = self.write_init_value_to_status_register().await;

            if status < 0 {
                return status;
            }

            read_words_from_register::<I2c, FRAME_SIZE>(&mut self.device, self.address, RAM_START_REGISTER, frame_data).await;

            status_word = read_word_from_register(&mut self.device, self.address, STATUS_REGISTER).await;

            data_ready = (status_word >> 3) & 0b1;
        }

        let control_word = read_word_from_register(&mut self.device, self.address, CONTROL_REGISTER).await;

        frame_data[832] = control_word;
        frame_data[833] = status_word & 0x0001;

        status = Self::validate_frame_data(frame_data);
        if status != 0
        {
            return status;
        }

        return frame_data[833] as i32;
    }

    pub async fn get_refresh_rate(&mut self) -> RefreshRate {
        let word = read_word_from_register(&mut self.device, self.address, CONTROL_REGISTER).await;

        RefreshRate::try_from((word >> 7 & 0b111) as u8).unwrap()
    }

    pub async fn set_refresh_rate(&mut self, refresh_rate: RefreshRate) {
        let control_word = read_word_from_register(&mut self.device, self.address, CONTROL_REGISTER).await;
        let new_refresh_rate = (refresh_rate as u16) << 7;
        let new_control_word = new_refresh_rate | (control_word & 0xFC7F);  // 0b1111110001111111, the 3 bit set to 0 to merge | with new refresh-rate

        write_word_to_register(&mut self.device, self.address, CONTROL_REGISTER, new_control_word).await;
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

    fn calculate_to(&mut self, frame_data: &[u16], emissivity: f32, tr: f32, result: &mut [f32]) {
        let mut ir_data_cp = [0.0f32; 2];
        let mut alpha_corr_r = [0.0f32; 4];

        let sub_page = frame_data[833];
        let vdd = self.get_vdd(frame_data);
        let ta = self.get_ta(frame_data);

        let mut ta4 = ta + 273.15;
        ta4 *= ta4;
        ta4 *= ta4;
        let mut tr4 = tr + 273.15;
        tr4 *= tr4;
        tr4 *= tr4;
        let ta_tr = tr4 - (tr4 - ta4) / emissivity;

        let kta_scale = 2.0f32.powf(self.params_mlx.kta_scale as f32);
        let kv_scale = 2.0f32.powf(self.params_mlx.kv_scale as f32);
        let alpha_scale = 2.0f32.powf(self.params_mlx.alpha_scale as f32);

        alpha_corr_r[0] = 1.0 / (1.0 + self.params_mlx.ks_to[0] * 40.0);
        alpha_corr_r[1] = 1.0;
        alpha_corr_r[2] = 1.0 + self.params_mlx.ks_to[1] * (self.params_mlx.ct[2] as f32);
        alpha_corr_r[3] = alpha_corr_r[2] * (1.0 + self.params_mlx.ks_to[2] * (self.params_mlx.ct[3] - self.params_mlx.ct[2]) as f32);

        let gain = ((self.params_mlx.gain_ee) / frame_data[778] as i16) as f32;

        let mode = ((frame_data[832] & 0x1000) >> 5) as u8;

        ir_data_cp[0] = frame_data[776] as i16 as f32 * gain;
        ir_data_cp[1] = frame_data[808] as i16 as f32 * gain;

        ir_data_cp[0] -= (self.params_mlx.cp_offset[0] as f32) * (1.0 + self.params_mlx.cp_kta * (ta - 25.0)) * (1.0 + self.params_mlx.cp_kv * (vdd - 3.3));
        if mode == self.params_mlx.calibration_mode_ee {
            ir_data_cp[1] -= (self.params_mlx.cp_offset[1] as f32) * (1.0 + self.params_mlx.cp_kta * (ta - 25.0)) * (1.0 + self.params_mlx.cp_kv * (vdd - 3.3));
        } else {
            ir_data_cp[1] -= ((self.params_mlx.cp_offset[1] as f32) + self.params_mlx.il_chess_c[0]) * (1.0 + self.params_mlx.cp_kta * (ta - 25.0)) * (1.0 + self.params_mlx.cp_kv * (vdd - 3.3));
        }

        for pixel_number in 0..TOT_PIXELS {
            let il_pattern = (pixel_number / 32 - pixel_number / 64 * 2) as i8;
            let chess_pattern = il_pattern.pow((pixel_number - pixel_number / 2 * 2) as u32);
            let conversion_pattern = (
                ((pixel_number + 2) / 4 - (pixel_number + 3) / 4 + (pixel_number + 1) / 4 - pixel_number / 4) as f32 * (1.0 - 2.0 * il_pattern as f32)
            ) as i8;

            let pattern = if mode == 0 { il_pattern } else { chess_pattern };

            if pattern != frame_data[833] as i8 { continue; }

            let mut ir_data = frame_data[pixel_number] as i16 as f32 * gain;

            let kta = (self.params_mlx.kta[pixel_number] as f32) / kta_scale;
            let kv = (self.params_mlx.kv[pixel_number] as f32) / kv_scale;
            ir_data -= (self.params_mlx.offset[pixel_number] as f32) * (1.0 + kta * (ta - 25.0)) * (1.0 + kv * (vdd - 3.3));

            if mode != self.params_mlx.calibration_mode_ee {
                ir_data += self.params_mlx.il_chess_c[2] * (2.0 * (il_pattern as f32) - 1.0) - self.params_mlx.il_chess_c[1] * (conversion_pattern as f32);
            }

            ir_data -= self.params_mlx.tgc * ir_data_cp[sub_page as usize];
            ir_data /= emissivity;

            let mut alpha_compensated = SCALE_ALPHA * alpha_scale / self.params_mlx.alpha[pixel_number] as f32;
            alpha_compensated *= 1.0 + self.params_mlx.ks_ta * (ta - 25.0);

            let mut sx = alpha_compensated.powi(3) * (ir_data + alpha_compensated * ta_tr);
            sx = sx.sqrt().sqrt() * self.params_mlx.ks_to[1];

            let mut to = (ir_data / (alpha_compensated * (1.0 - self.params_mlx.ks_to[1] * 273.15) + sx) + ta_tr).sqrt().sqrt() - 273.15;

            let range = if to < self.params_mlx.ct[1] as f32 {
                0
            } else if to < self.params_mlx.ct[2] as f32 {
                1
            } else if to < self.params_mlx.ct[3] as f32 {
                2
            } else {
                3
            };

            to = (ir_data / (alpha_compensated * alpha_corr_r[range] * (1.0 + self.params_mlx.ks_to[range] * (to - self.params_mlx.ct[range] as f32))) + ta_tr).sqrt().sqrt() - 273.15;

            result[pixel_number] = to;
        }
    }

    fn get_ta(&self, frame_data: &[u16]) -> f32 {
        let vdd = self.get_vdd(frame_data);
        let ptat = frame_data[800] as i16;

        let ptat_art = (
            (ptat as f32) / ((ptat as f32 * self.params_mlx.alpha_ptat) + (frame_data[768] as i16 as f32)) * 2f32.powf(18.0)
        ) as i16 as f32;

        let mut ta = ptat_art / (1.0 + self.params_mlx.kv_ptat * (vdd - 3.3)) - (self.params_mlx.vp_tat25 as f32);
        ta /= self.params_mlx.kt_ptat;
        ta += 25.0;

        ta
    }

    fn get_vdd(&self, frame_data: &[u16]) -> f32 {
        let vdd = frame_data[810] as i16;

        let resolution_ram: i32 = ((frame_data[832] & 0x0C00) >> 10) as i32;
        let resolution_correction = (2f32.powf(self.params_mlx.resolution_ee as f32)) / 2f32.powf(resolution_ram as f32);

        (resolution_correction * (vdd as f32) - (self.params_mlx.vdd25 as f32)) / self.params_mlx.k_vdd as f32 + 3.3
    }

    async fn write_init_value_to_status_register(&mut self) -> i32 {
        let init_word: u16 = 0x0030;
        let sub_page0check = 0x10;
        let sub_page1check = 0x11;

        write_word_to_register(&mut self.device, self.address, STATUS_REGISTER, init_word).await;

        let data_check = read_word_from_register(&mut self.device, self.address, STATUS_REGISTER).await;
        if data_check == sub_page0check || data_check == sub_page1check {
            return data_check as i32;
        }

        return -2;
    }
}

pub struct ParamsMlx {
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
    pub async fn init_parameters<I2c>(&mut self, device: &mut I2c, address: u8)
    where
        I2c: embedded_hal_async::i2c::I2c,
    {
        let mut eeprom_data = [0u16; 832];
        read_words_from_register::<I2c, 832>(device, address, EE_PROM_START_ADDRESS, &mut eeprom_data).await;
        self.extract_vdd_parameters(&eeprom_data);
        self.extract_ptat_parameters(&eeprom_data);
        self.extract_gain_parameters(&eeprom_data);
        self.extract_tgc_parameters(&eeprom_data);
        self.extract_resolution_parameters(&eeprom_data);
        self.extract_ks_ta_parameters(&eeprom_data);
        self.extract_ks_to_parameters(&eeprom_data);
        self.extract_alpha_parameters(&eeprom_data);
        self.extract_offset_parameters(&eeprom_data);
        self.extract_kta_pixel_parameters(&eeprom_data);
        self.extract_kv_pixel_parameters(&eeprom_data);
        self.extract_cp_parameters(&eeprom_data);
        self.extract_cilc_parameters(&eeprom_data);
        self.extract_deviating_pixels(&eeprom_data);
        self.alpha_scale = 10u8;
    }

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
        self.tgc = (ee_data[60] as i8 & 0xFFu8 as i8) as f32 / 32.0;
    }

    fn extract_resolution_parameters(&mut self, ee_data: &[u16]) {
        self.resolution_ee = ((ee_data[56] & 0x3000) >> 12) as u8;
    }

    fn extract_ks_ta_parameters(&mut self, ee_data: &[u16]) {
        self.ks_ta = (((ee_data[60] & 0xFF00) >> 8) as i8) as f32 / 8192.0;
    }

    fn extract_ks_to_parameters(&mut self, ee_data: &[u16]) {
        let step: i16 = (((ee_data[63] & 0x3000) >> 12) * 10) as i16;

        self.ct[0] = -40;
        self.ct[1] = 0;
        self.ct[2] = ((ee_data[63] & 0x00F0) >> 4) as i16;
        self.ct[3] = ((ee_data[63] & 0x0F00) >> 8) as i16;

        self.ct[2] *= step;
        self.ct[3] = self.ct[2] + self.ct[3] * step;
        self.ct[4] = 400;

        let ks_to_scale = 1 << ((ee_data[63] & 0x000F) + 8);

        self.ks_to[0] = (ee_data[61] & 0x00FF) as i8 as f32 / ks_to_scale as f32;
        self.ks_to[1] = ((ee_data[61] >> 8) & 0x00FF) as i8 as f32 / ks_to_scale as f32;
        self.ks_to[2] = (ee_data[62] & 0x00FF) as i8 as f32 / ks_to_scale as f32;
        self.ks_to[3] = ((ee_data[62] >> 8) & 0x00FF) as i8 as f32 / ks_to_scale as f32;
        self.ks_to[4] = -0.0002;
    }

    fn extract_alpha_parameters(&mut self, ee_data: &[u16]) {
        let mut acc_row: [i32; 24] = [0; 24];
        let mut acc_column: [i32; 32] = [0; 32];
        let mut p: usize;
        let mut alpha_temp: [f32; TOT_PIXELS] = [0.0; TOT_PIXELS];

        let acc_rem_scale: u8 = (ee_data[32] & 0x000F) as u8;
        let acc_column_scale: u8 = ((ee_data[32] & 0x00F0) >> 4) as u8;
        let acc_row_scale: u8 = ((ee_data[32] & 0x0F00) >> 8) as u8;
        let alpha_ref: i32 = ee_data[33] as i32;

        for i in 0..6 {
            p = i * 4;
            acc_row[p + 0] = (ee_data[34 + i] & 0x00F) as i32;
            acc_row[p + 1] = ((ee_data[34 + i] & 0x00F0) >> 4) as i32;
            acc_row[p + 2] = ((ee_data[34 + i] & 0x0F00) >> 8) as i32;
            acc_row[p + 3] = ((ee_data[34 + i] & 0xf000) >> 12) as i32;
        }

        for i in 0..TOT_ROWS {
            if acc_row[i] > 7 {
                acc_row[i] -= 16;
            }
        }

        for i in 0..8 {
            p = i * 4;
            acc_column[p + 0] = (ee_data[40 + i] & 0x000F) as i32;
            acc_column[p + 1] = ((ee_data[40 + i] & 0x00F0) >> 4) as i32;
            acc_column[p + 2] = ((ee_data[40 + i] & 0x0F00) >> 8) as i32;
            acc_column[p + 3] = ((ee_data[40 + i] & 0xF000) >> 12) as i32;
        }

        for i in 0..TOT_COLUMNS {
            if acc_column[i] > 7 {
                acc_column[i] -= 16;
            }
        }

        for i in 0..TOT_ROWS {
            for j in 0..TOT_COLUMNS {
                p = 32 * i + j;
                alpha_temp[p] = ((ee_data[64 + p] & 0x03F0) >> 4) as f32;
                if alpha_temp[p] > 31.0 {
                    alpha_temp[p] -= 64.0;
                }

                alpha_temp[p] *= (1 << acc_rem_scale) as f32;
                alpha_temp[p] = alpha_ref as f32 +
                    (acc_row[i] << acc_row_scale) as f32 +
                    (acc_column[j] << acc_column_scale) as f32 +
                    alpha_temp[p];
                alpha_temp[p] /= f32::powf(2.0, 4.0);
                alpha_temp[p] -= self.tgc * (self.cp_alpha[0] + self.cp_alpha[1]) / 2.0;
                alpha_temp[p] = SCALE_ALPHA / alpha_temp[p];
            }
        }

        let mut temp: f32 = alpha_temp[0];
        for i in 0..TOT_PIXELS {
            if alpha_temp[i] > temp {
                temp = alpha_temp[i];
            }
        }

        let mut alpha_scale: u8 = 0;
        while temp < 32767.4 {
            temp *= 2.0;
            alpha_scale += 1;
        }

        for i in 0..TOT_PIXELS {
            temp = alpha_temp[i] * f32::powf(2.0, alpha_scale as f32);
            self.alpha[i] = (temp + 0.5) as u16;
        }

        self.alpha_scale = alpha_scale;
    }

    fn extract_offset_parameters(&mut self, ee_data: &[u16]) {
        let mut occ_row: [i32; 24] = [0; 24];
        let mut occ_column: [i32; 32] = [0; 32];
        let mut p: usize;

        let occ_rem_scale = (ee_data[16] & 0x000F) as u8;
        let occ_column_scale = ((ee_data[16] & 0x00F0) >> 4) as u8;
        let occ_row_scale = ((ee_data[16] & 0x0F00) >> 8) as u8;
        let offset_ref = ee_data[17] as i16;

        for i in 0..6 {
            p = i * 4;
            occ_row[p + 0] = (ee_data[18] & 0x000F) as i32;
            occ_row[p + 1] = ((ee_data[18] & 0x00F0) >> 4) as i32;
            occ_row[p + 2] = ((ee_data[18 + i] & 0x0F00) >> 8) as i32;
            occ_row[p + 3] = ((ee_data[18 + i] & 0xF000) >> 12) as i32;
        }

        for i in 0..TOT_ROWS {
            if occ_row[i] > 7 {
                occ_row[i] -= 16;
            }
        }

        for i in 0..8 {
            p = i * 4;
            occ_column[p + 0] = (ee_data[24 + i] & 0x000F) as i32;
            occ_column[p + 1] = ((ee_data[24 + i] & 0x00F0) >> 4) as i32;
            occ_column[p + 2] = ((ee_data[24 + i] & 0x0F00) >> 8) as i32;
            occ_column[p + 3] = ((ee_data[24 + i] & 0xF000) >> 12) as i32;
        }

        for i in 0..TOT_COLUMNS {
            if occ_column[i] > 7 {
                occ_column[i] -= 16;
            }
        }

        for i in 0..TOT_ROWS {
            for j in 0..TOT_COLUMNS {
                p = 32 * i + j;
                self.offset[p] = ((ee_data[64 + p] & 0xFC00) >> 10) as i16;
                if self.offset[p] > 31 {
                    self.offset[p] -= 64;
                }
                self.offset[p] *= (1 << occ_rem_scale) as i16;
                let row_plus_scale = (occ_row[i] << occ_row_scale) as i16;
                let cols_plus_scale = (occ_column[j] << occ_column_scale) as i16;
                self.offset[p] = offset_ref + row_plus_scale + cols_plus_scale + self.offset[p];
            }
        }
    }

    fn extract_kta_pixel_parameters(&mut self, ee_data: &[u16]) {
        let mut kta_rc = [0i8; 4];
        let mut kta_temp = [0f32; 768];

        kta_rc[0] = ((ee_data[54] & 0xFF00) >> 8) as i8;
        kta_rc[2] = (ee_data[54] & 0x00FF) as i8;
        kta_rc[1] = ((ee_data[55] & 0xFF00) >> 8) as i8;
        kta_rc[3] = (ee_data[55] & 0x00FF) as i8;

        let mut kta_scale1 = ((ee_data[56] & 0x00F0) >> 4) as u8 + 8;
        let kta_scale2 = (ee_data[56] & 0x000F) as u8;

        for i in 0..TOT_ROWS {
            for j in 0..TOT_COLUMNS {
                let p = 32 * i + j;
                let split = 2 * ((p / 32) - (p / 64) * 2) + (p % 2);
                let mut temp_val = ((ee_data[64 + p] & 0x000E) >> 1) as f32;
                if temp_val > 3.0 {
                    temp_val -= 8.0;
                }

                temp_val *= (1 << kta_scale2) as f32;
                temp_val = kta_rc[split] as f32 + temp_val;
                kta_temp[p] = temp_val / 2f32.powi(kta_scale1 as i32);
            }
        }

        let mut temp = kta_temp[0].abs();
        for i in 1..TOT_PIXELS {
            let abs_val = kta_temp[i].abs();
            if abs_val > temp {
                temp = abs_val;
            }
        }

        kta_scale1 = 0;
        while temp < 63.4 {
            temp *= 2.0;
            kta_scale1 += 1;
        }

        for i in 0..TOT_PIXELS {
            let val = kta_temp[i] * 2f32.powi(kta_scale1 as i32);
            self.kta[i] = if val < 0.0 {
                (val - 0.5).floor() as i8
            } else {
                (val + 0.5).floor() as i8
            };
        }

        self.kta_scale = kta_scale1;
    }

    fn extract_kv_pixel_parameters(&mut self, ee_data: &[u16]) {
        let mut kv_t = [0i8; 4];
        let mut kv_temp = [0f32; TOT_PIXELS];

        let mut kv_ro_co = ((ee_data[52] & 0xF000) >> 12) as i8;
        if kv_ro_co > 7 {
            kv_ro_co -= 16;
        }
        kv_t[0] = kv_ro_co;

        let mut kv_re_co = ((ee_data[52] & 0x0F00) >> 8) as i8;
        if kv_re_co > 7 {
            kv_re_co -= 16;
        }
        kv_t[2] = kv_re_co;

        let mut kv_ro_ce = ((ee_data[52] & 0x00F0) >> 4) as i8;
        if kv_ro_ce > 7 {
            kv_ro_ce -= 16;
        }
        kv_t[1] = kv_ro_ce;

        let mut kv_re_ce = (ee_data[52] & 0x000F) as i8;
        if kv_re_ce > 7 {
            kv_re_ce -= 16;
        }
        kv_t[3] = kv_re_ce;

        let mut kv_scale = ((ee_data[56] & 0x0F00) >> 8) as u8;

        for i in 0..TOT_ROWS {
            for j in 0..TOT_COLUMNS {
                let p = 32 * i + j;
                let split = 2 * ((p / 32) - (p / 64) * 2) + (p % 2);
                kv_temp[p] = kv_t[split] as f32;
                kv_temp[p] /= 2f32.powi(kv_scale as i32);
            }
        }

        let mut temp = kv_temp[0].abs();
        for i in 1..TOT_PIXELS {
            let abs_val = kv_temp[i].abs();
            if abs_val > temp {
                temp = abs_val;
            }
        }

        kv_scale = 0;
        while temp < 63.4 {
            temp *= 2.0;
            kv_scale += 1;
        }

        for i in 0..TOT_PIXELS {
            temp = kv_temp[i] * 2f32.powi(kv_scale as i32);
            self.kv[i] = if temp < 0.0 {
                (temp - 0.5) as i8
            } else {
                (temp + 0.5) as i8
            };
        }

        self.kv_scale = kv_scale;
    }

    fn extract_cp_parameters(&mut self, ee_data: &[u16]) {
        let mut alpha_sp = [0.0f32; 2];
        let mut offset_sp = [0i16; 2];

        let alpha_scale = ((ee_data[32] & 0xF000) >> 12) + 27;

        offset_sp[0] = (ee_data[58] & 0x03FF) as i16;
        if offset_sp[0] > 511 {
            offset_sp[0] -= 1024;
        }

        offset_sp[1] = ((ee_data[58] & 0xFC00) >> 10) as i16;
        if offset_sp[1] > 31 {
            offset_sp[1] -= 64;
        }
        offset_sp[1] += offset_sp[0];

        alpha_sp[0] = (ee_data[57] & 0x03FF) as f32;
        if alpha_sp[0] > 511.0 {
            alpha_sp[0] -= 1024.0;
        }

        alpha_sp[0] /= 2f32.powi(alpha_scale as i32);

        alpha_sp[1] = ((ee_data[57] & 0xFC00) >> 10) as f32;
        if alpha_sp[1] > 31.0 {
            alpha_sp[1] -= 64.0;
        }

        alpha_sp[1] = (1.0 + alpha_sp[1] / 128.0) * alpha_sp[0];

        let cp_kta = (ee_data[59] & 0x00FF) as i8;

        let kta_scale1: i32 = (((ee_data[56] & 0x00F0) >> 4) + 8) as i32;
        self.cp_kta = cp_kta as f32 / 2f32.powi(kta_scale1);

        let cp_kv = ((ee_data[59] & 0xFF00) >> 8) as i8;

        let kv_scale = ((ee_data[56] & 0x0F00) >> 8) as i32;
        self.cp_kv = cp_kv as f32 / 2f32.powi(kv_scale);

        self.cp_alpha[0] = alpha_sp[0];
        self.cp_alpha[1] = alpha_sp[1];
        self.cp_offset[0] = offset_sp[0];
        self.cp_offset[1] = offset_sp[1];
    }

    fn extract_cilc_parameters(&mut self, ee_data: &[u16]) {
        let mut il_chess_c = [0.0f32; 3];

        let mut calibration_mode_ee = ((ee_data[10] & 0x0800) >> 4) as u8;
        calibration_mode_ee ^= 0x80;

        il_chess_c[0] = (ee_data[53] & 0x003F) as f32;
        if il_chess_c[0] > 31.0 {
            il_chess_c[0] -= 64.0;
        }
        il_chess_c[0] /= 16.0;

        il_chess_c[1] = ((ee_data[53] & 0x07C0) >> 6) as f32;
        if il_chess_c[1] > 15.0 {
            il_chess_c[1] -= 32.0;
        }
        il_chess_c[1] /= 2.0;

        il_chess_c[2] = ((ee_data[53] & 0xF800) >> 11) as f32;
        if il_chess_c[2] > 15.0 {
            il_chess_c[2] -= 32.0;
        }
        il_chess_c[2] /= 8.0;

        self.calibration_mode_ee = calibration_mode_ee;
        self.il_chess_c[0] = il_chess_c[0];
        self.il_chess_c[1] = il_chess_c[1];
        self.il_chess_c[2] = il_chess_c[2];
    }

    fn extract_deviating_pixels(&mut self, ee_data: &[u16]) -> i32 {
        let mut pix_cnt: usize;
        let mut broken_pix_cnt: usize = 0;
        let mut outlier_pix_cnt: usize = 0;
        let mut warn = 0;

        for i in 0..5 {
            self.broken_pixels[i] = 0xFFFF;
            self.outlier_pixels[i] = 0xFFFF;
        }

        pix_cnt = 0;
        while pix_cnt < TOT_PIXELS && broken_pix_cnt < 5 && outlier_pix_cnt < 5 {
            let pixel_val = ee_data[pix_cnt + 64];
            if pixel_val == 0 {
                self.broken_pixels[broken_pix_cnt] = pix_cnt as u16;
                broken_pix_cnt += 1;
            } else if (pixel_val & 0x0001) != 0 {
                self.outlier_pixels[outlier_pix_cnt] = pix_cnt as u16;
                outlier_pix_cnt += 1;
            }

            pix_cnt += 1;
        }

        if broken_pix_cnt > 4 {
            warn = -3;
        } else if outlier_pix_cnt > 4 {
            warn = -4;
        } else if (broken_pix_cnt + outlier_pix_cnt) > 4 {
            warn = -5;
        } else {
            for i in 0..broken_pix_cnt {
                for j in (i + 1)..broken_pix_cnt {
                    warn = self.check_adjacent_pixels(self.broken_pixels[i], self.broken_pixels[j]);
                    if warn != 0 {
                        return warn;
                    }
                }
            }

            for i in 0..outlier_pix_cnt {
                for j in (i + 1)..outlier_pix_cnt {
                    warn = self.check_adjacent_pixels(self.outlier_pixels[i], self.outlier_pixels[j]);
                    if warn != 0 {
                        return warn;
                    }
                }
            }

            for i in 0..broken_pix_cnt {
                for j in 0..outlier_pix_cnt {
                    warn = self.check_adjacent_pixels(self.broken_pixels[i], self.outlier_pixels[j]);
                    if warn != 0 {
                        return warn;
                    }
                }
            }
        }

        warn
    }

    fn check_adjacent_pixels(&self, pix1: u16, pix2: u16) -> i32 {
        let lp1 = pix1 >> 5;
        let lp2 = pix2 >> 5;
        let cp1 = pix1 - (lp1 << 5);
        let cp2 = pix2 - (lp2 << 5);

        let mut pix_pos_dif = lp1 as i16 - lp2 as i16;
        if pix_pos_dif <= -2 || pix_pos_dif >= 2 {
            return 0;
        }

        pix_pos_dif = cp1 as i16 - cp2 as i16;
        if pix_pos_dif > -2 && pix_pos_dif < 2 {
            return -6;
        }

        0
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