use std::io;
use linux_thermal_camera_mlx90640::{RefreshRate, ThermalCamera};

fn main() {
    let mut thermal_camera = ThermalCamera::default();

    println!("i2c bus initialised successfully");

    let refresh_rate = thermal_camera.get_refresh_rate();

    println!("current refresh rate {:?}", refresh_rate);

    let mut line = String::from("");
    let mut new_refresh_rate = 255u8;

    while new_refresh_rate == 255 {
        println!("insert desired refresh-rate to set [0, 1, 2, 3, 4, 5, 6, 7]:");
        match io::stdin().read_line(&mut line) {
            Ok(_) => {
                let num = line.trim().parse::<u8>();
                if num.is_err() {
                    println!("failed to parse number {}", line);
                    line = String::from("");
                    continue;
                }
                new_refresh_rate = num.unwrap();
                match RefreshRate::try_from(new_refresh_rate) {
                    Ok(r) => {
                        thermal_camera.set_refresh_rate(r);
                        println!("new refresh-rate successfully set");
                        break;
                    }
                    Err(_) => {
                        new_refresh_rate = 255;
                        line = String::from("");
                        println!("failed to parse refresh-rate, values allowed are [0, 1, 2, 3, 4, 5, 6, 7]")
                    }
                }
            }
            Err(e) => {
                println!("error while reading from stdin {e}")
            }
        };

    }

    let new_refresh_rate = thermal_camera.get_refresh_rate();

    println!("new refresh rate {:?}", new_refresh_rate);

}