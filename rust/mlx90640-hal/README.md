# Mlx90640 HAL

A Hardware abstraction layer to query and run the mlx90640 infrared sensor on several platform and devices, as long as the `I2C embedded_hal` is implemented correctly.

### No heap allocation
Using the `heapless` library, we can allocate on the stack the fixed-size vectors responsible for initialising the parameters and querying the thermal frame.

### Cross Platform thanks to `no_std`
tested successfully on the following devices:
* Raspberry Pi Zero 2 W
* Raspberry Pi Pico (rp2040, using the embassy runtime)

### Build Command
To build the library and the example provided run:
```shell
cargo build --release --examples
```