# MLX90640 .NET driver

[MIT Licence](LICENCE-MIT)

This project implements a thermal camera .NET driver to interface the MLX90640 infra-red sensor in order to configure and fetch the
thermal frame.

## Usage
```csharp
// Initialize the device
using var thermalCamera = new ThermalCamera();

// Set desired refresh rate (2Hz which corresponds to 1fps)
await thermalCamera.SetRefreshRate(ThermalCamera.RefreshRate._2Hz);

// Get current frame
var image = await thermalCamera.GetImage();
      
// Process frame based on your needs  
```
### How to install this library in your .NET project
```shell
dotnet add package ThermalCameraMlx90640 --version 1.0.0
```

### Compile your project for linux arm64
#### (or 32 bit, based on your device architecture)
```shell
dotnet publish --runtime linux-arm64 --self-contained  
```
