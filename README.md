# thermal-camera-MLX90640
This repo contains drivers to interface MLX90640 infra-red array sensor in order to get current the thermal frame and configure certain parameters.

### Demo
There's a minimal [React App](thermal-camera-viewer) for receiving frame through [websocket](rust/camera-ws-stream) and for rendering the thermal frame using a **canvas**
![Demo](rust-thermal-viewer.gif)

## Content List
* [MLX90640 datasheet](docs/MLX90640-Datasheet-Melexis.pdf)
* [MLX90640 driver documentation](docs/MLX90640%20driver.pdf)
* [.NET driver](dotnet/ThermalCameraMlx90640/ThermalCameraMlx90640) (using the Microsoft IoT I²C driver )
* [Rust driver](rust/linux-mlx90640) (using the Linux I²C driver thanks to [rust-i2cdev](https://github.com/rust-embedded/rust-i2cdev))

### Current limitations on .Net
- refresh-rate limited to 4Hz due to inexplicable huge delay (150ms) when reading from RAM registers:
  - there must be a timing issue with the Microsoft library when querying the ram registers at high baud-rate.


## Support the project  🙏🏼 ⭐
If you’ve found this project helpful or enjoy using it, I’d be incredibly grateful for your support! Your contributions help keep the development going and ensure continued improvements. If you'd like to show your appreciation, consider making a donation:

- [<img src="paypal.png" width="32">](https://paypal.me/fragrama17?country.x=IT&locale.x=it_IT) paypal [link](https://paypal.me/fragrama17?country.x=IT&locale.x=it_IT)
- <img src="westpac.png" width="32"> BSB: 732-250, Account Number: 754415
- <img src="sepa.jpeg" width="80"> IBAN: IT55 T036 4601 6005 2650

or even a star would be greatly appreciated ⭐

## Thanks
Special thanks to [Melexis](https://github.com/melexis) for providing the MLX90640 sensor documentation and [Adafruit](https://github.com/adafruit) for offering valuable templates and resources that helped inspire and guide the development of this driver.
