use linux_thermal_camera_mlx90640::ThermalCamera;

fn main() {
    println!("and here we go, doing awesome things in rust ! ;D");
    let mut thermal_camera = ThermalCamera::new(0x33, 1);

    println!("i2c bus initialised successfully");

    let refresh_rate = thermal_camera.get_refresh_rate();

    println!("current refresh rate {:?}", refresh_rate);

    // println!("trying to get the image");

    // let image = thermal_camera.get_image();

    // for pixel in image{

    //     println!("{}", pixel)

    // }
}