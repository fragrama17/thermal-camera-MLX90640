use linux_thermal_camera_mlx90640::{RefreshRate, ThermalCamera};

fn main() {
    println!("and here we go, doing awesome things in rust ! ;D");
    let mut thermal_camera = ThermalCamera::new(0x33, 1);

    println!("i2c bus initialised successfully");

    let refresh_rate = thermal_camera.get_refresh_rate();

    println!("current refresh rate {:?}", refresh_rate);

    thermal_camera.set_refresh_rate(RefreshRate::_4Hz);

    let new_refresh_rate = thermal_camera.get_refresh_rate();

    println!("new refresh rate {:?}", new_refresh_rate);

}