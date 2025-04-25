use std::time::Instant;
use linux_thermal_camera_mlx90640::ThermalCamera;

fn main() {
    let mut camera = ThermalCamera::new(0x33, 1);

    camera.init_parameters();

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