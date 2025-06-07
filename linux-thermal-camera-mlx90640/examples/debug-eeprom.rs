use linux_thermal_camera_mlx90640::{ThermalCamera};

fn main() {

    let mut cam = ThermalCamera::new(0x33, 1);
    cam.init_parameters();

    println!("{:?}", cam.params_mlx);

}