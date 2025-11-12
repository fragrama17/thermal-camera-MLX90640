#![no_std]
#![no_main]

use defmt::*;

use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts};
use embassy_rp::i2c::{Config, I2c, InterruptHandler};
use embassy_rp::peripherals::I2C1;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
use mlx90640_async::{RefreshRate, ThermalCamera};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {

    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    info!("set up i2c ");
    let i2c = I2c::new_async(p.I2C1, scl, sda, Irqs, Config::default());

    let mut tc = ThermalCamera::new(0x33, i2c);

    tc.set_refresh_rate(RefreshRate::_2Hz).await;

    info!("refresh rate set to 2hz");

    Timer::after_secs(3).await;
    
    loop {
        let frame = tc.get_image().await;

        match frame {
            Ok(f) => {
                print_frame(&f)
            }
            Err(e) => {
                error!("failed to retrieve thermal frame");
            }
        }
    }
}

fn print_frame(frame: &[f32; 768]) {
    let rows = 24;
    let cols = 32;

    info!("[");
    for r in 0..rows {
        for c in 0..cols {
            info!("{} ", frame[r * cols + c]); // Adjust formatting as needed
        }
    }
    info!("]");
}