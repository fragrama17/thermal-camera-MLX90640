// use rand::Rng;
use linux_thermal_camera_mlx90640::ThermalCamera;
// fn fill_with_random<const N: usize>(arr: &mut [u8; N]) {
//     let mut rng = rand::thread_rng();
//
//     for i in 0..N {
//         arr[i] = rng.gen();
//     }
// }
//
// fn main() {
//     let mut arr = [0u8; 10]; // Array with 10 elements, initially all zeroes
//     fill_with_random(&mut arr);
//
//     println!("{:?}", arr); // Prints the array with random values
// }


fn main() {
    println!("and here we go, doing awesome things in rust ! ;D");
    let mut thermal_camera = ThermalCamera::new(0x33, 1);

    println!("i2c bus initialised successfully");

    let refresh_rate = thermal_camera.get_refresh_rate();

    println!("current refresh rate {:?}", refresh_rate);

    let mut frame_data = [0u16; 834];

    println!("trying to get the image");

    let image = thermal_camera.get_image(&mut frame_data);

    for &pixel in image{

        println!("{}", pixel)

    }
}