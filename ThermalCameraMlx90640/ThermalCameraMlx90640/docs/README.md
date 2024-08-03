# MLX90640 .NET driver

This project implements a thermal camera .NET driver to interface the MLX90640 infra-red sensor in order to configure and fetch the
thermal frame.

## Usage
```
// Initialize the device
using var thermalCamera = new ThermalCamera();

// Set desired refresh rate (2Hz which corresponds to 1fps)
await thermalCamera.SetRefreshRate(ThermalCamera.RefreshRate._2_Hz);

// Get current frame
var image = await thermalCamera.GetImage();
      
// Process frame based on your needs     
```
