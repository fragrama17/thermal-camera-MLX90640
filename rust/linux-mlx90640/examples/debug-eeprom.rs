use linux_mlx90640::{ThermalCamera};

fn main() {

    let cam = ThermalCamera::default();

    println!("{:?}", cam.params_mlx);

}