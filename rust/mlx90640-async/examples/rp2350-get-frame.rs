#![no_std]
#![no_main]

use defmt::*;

use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts};
use embassy_rp::i2c::{Config, I2c, InterruptHandler};
use embassy_rp::peripherals::I2C1;
use embassy_time::Delay;
use {defmt_rtt as _, panic_probe as _};
use mlx90640_async::{Device, RefreshRate, ThermalCamera};
use libm::{powf};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {

    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    let pi = 3.141592653;
    debug!("square of Pi is: {}", powf(pi, 2.0));

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    info!("setting up i2c");
    let mut c = Config::default();
    c.frequency = 1_000_000;
    let i2c = I2c::new_async(p.I2C1, scl, sda, Irqs, c);

    let mut tc = ThermalCamera::new(0x33, i2c, Delay).await;

    tc.set_refresh_rate(RefreshRate::_2Hz).await;

    info!("refresh rate set to 2hz");

    debug!("{}", tc.params_mlx.to_string().as_str());
    
    loop {
        let frame = tc.get_image().await;

        match frame {
            Ok(f) => {
                print_frame(&f)
            }
            Err(e) => {
                error!("failed to retrieve thermal frame {}", e);
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