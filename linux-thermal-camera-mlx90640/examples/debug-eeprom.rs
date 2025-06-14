use linux_thermal_camera_mlx90640::{ThermalCamera};

fn main() {

    let cam = ThermalCamera::default();

    println!("{:?}", cam.params_mlx);

}