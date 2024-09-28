use linux_thermal_camera_mlx90640::ThermalCamera;

fn main() {
    ThermalCamera::new(0x33, 1);
}