#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::i2c::{Config, I2c};
use embassy_time::Delay;
use mlx90640_hal::{Mlx90640, RefreshRate, TOT_PIXELS};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let i2c_config = Config::default();

    let i2c = I2c::new_blocking(
        p.I2C1,
        p.PIN_27,
        p.PIN_26,
        i2c_config,
    );
    let mut camera = Mlx90640::new(0x33, i2c, Delay);
    camera.set_refresh_rate(RefreshRate::_2Hz).unwrap();

    loop {
        match camera.get_image() {
            Ok(frame) => {
                debug_matrix(&frame)
            }
            Err(_) => {
                error!("error while querying thermal frame");
            }
        }
    }
}

pub fn debug_matrix(data: &[f32; TOT_PIXELS]) {
    const ROWS: usize = 24;
    const COLS: usize = 32;

    debug!("┌──────────────── 24 x 32 f32 matrix ────────────────");

    for row in 0..ROWS {
        debug!("│ row {}: {}", row, &data[row * COLS..(row + 1) * COLS]);
    }

    debug!("└────────────────────────────────────────────────────");
}