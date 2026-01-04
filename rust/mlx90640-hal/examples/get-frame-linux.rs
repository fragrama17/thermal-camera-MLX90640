use std::time::Instant;
use linux_embedded_hal::{Delay, I2cdev};
use mlx90640_hal::{Mlx90640, RefreshRate};

fn main() {
    let mut camera = Mlx90640::new(0x33, I2cdev::new("/dev/i2c-1").unwrap(), Delay);
    camera.set_refresh_rate(RefreshRate::_4Hz).unwrap();
    
    loop {
        let start = Instant::now();
        match camera.get_image() {
            Ok(frame) => {
                print_frame(&frame);
                println!("fetched matrix from camera in just {}ms", start.elapsed().as_millis());
            }
            Err(e) => {
                println!("{}", e)
            }
        };
    }
}

fn print_frame(frame: &[f32]) {
    let rows = 24;
    let cols = 32;
    assert_eq!(frame.len(), rows * cols, "Frame size does not match the given rows and cols");

    println!("[");
    for r in 0..rows {
        for c in 0..cols {
            print!("{:6.2} ", frame[r * cols + c]); // Adjust formatting as needed
        }
        println!(); // Move to the next line after each row
    }
    println!("]");
}