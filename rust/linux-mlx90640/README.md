# Linux MLX90640
A rust driver to easily interface mlx90640, running on linux arm32/64 using the linux I²C api.

## Quick Usage
The easiest way to use this library is by taking advantage of the default constructor as shown below ([example](./examples/get-frame.rs)):
```rust
use std::time::Instant;
use linux_mlx90640::ThermalCamera;

fn main() {
    let mut camera = ThermalCamera::default();

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
    // converting to jagged matrix
}
```
### Custom address/bus
If you need to specify a different I²C address, you can use the **new** method:
```rust
use linux_mlx90640::ThermalCamera;

fn main() {
    // ThermalCamera::new(address: u16, busId: i32)
    let mut camera = ThermalCamera::new(0x33, 1);
    //process image based on your needs
}
```

## How to compile
Make sure you have properly installed the **Arm GNU Toolchain** and that [linker is pointing to the compiler in config.toml](.cargo/config.toml).

Then run:
```shell
cargo build --release --examples
```