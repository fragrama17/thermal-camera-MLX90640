using System.Device.I2c;

namespace ThermalCamera;

public static class Program
{
    public static async Task Main(string[] args)
    {
        var thermalCamera = new ThermalCamera();
        // var frame = new float[ThermalCamera.TotPixels];
        // await thermalCamera.GetImage(frame);
        var frame = new ushort[834];
        var status = await ThermalCamera.GetFrameData(frame);
        Console.WriteLine(status >= 0 ? "all good" : "error while reading data frame !");
        Console.WriteLine("Thermal Frame:");
        Console.WriteLine("[");
        for (int i = 0; i < ThermalCamera.TotRows; i++)
        {
            // Console.WriteLine($"current row {i + 1}");
            Console.Write("     ");
            for (int j = 0; j < ThermalCamera.TotColumns; j++)
            {
                Console.Write($"{frame[i * ThermalCamera.TotColumns + j]}  ");
                // Console.Write($"{i * ThermalCamera.TotColumns + j}  ");
            }
            Console.WriteLine("");
        }

        Console.WriteLine("]");

        // const int statusReady = 0x8000;
        // const int refreshRateHz = 0x800D;
        //
        // var device = I2cBus.Create(1).CreateDevice(0x33);
        // Console.WriteLine("my freaking thermal-camera firmware written in C#");
        //
        // // STATUS, WORKING !!! :D
        // var statusBuffer = new byte[2];
        // var statusAddressBuffer = new byte[2];
        // statusAddressBuffer[0] = statusReady >> 8;
        // statusAddressBuffer[1] = statusReady & 0xFF;
        //
        // device.WriteRead(statusAddressBuffer, statusBuffer);
        //
        // var statusWord = (ushort)((statusBuffer[0] << 8) | statusBuffer[1]);
        // Console.WriteLine($"STATUS register, current value: 0b{Convert.ToString(statusWord, 2).PadLeft(16, '0')}");
        //
        // var extractedStatus = (statusWord >> 3) & 0b1;
        // var statusMsg = extractedStatus > 0 ? "data available in RAM" : "NO data available in RAM";
        // Console.WriteLine($"STATUS register, {statusMsg}");
        //
        // // REFRESH, WORKING !!! :D
        // var refreshBuffer = new byte[2];
        // var refreshAddressBuffer = new byte[2];
        // refreshAddressBuffer[0] = refreshRateHz >> 8; // MSB 
        // refreshAddressBuffer[1] = refreshRateHz & 0xFF; // LSB
        //
        // device.WriteRead(refreshAddressBuffer, refreshBuffer);
        //
        // var controlWord = (ushort)((refreshBuffer[0] << 8) | refreshBuffer[1]);
        // Console.WriteLine(
        //     $"REFRESH_RATE register, current value: 0b{Convert.ToString(controlWord, 2).PadLeft(16, '0')}");
        //
        // var extractedValue = (controlWord >> 7) & 0b111;
        // Console.WriteLine($"REFRESH_RATE register, current frequency: {RefreshRateToPrettyString(extractedValue)}");
    }

    private static string RefreshRateToPrettyString(int value)
    {
        return value switch
        {
            0b000 => "0.5Hz",
            0b001 => "1Hz",
            0b010 => "2Hz",
            0b011 => "4Hz",
            0b100 => "8Hz",
            0b101 => "16Hz",
            0b110 => "32Hz",
            0b111 => "64Hz",
            _ => "UNKNOWN"
        };
    }
}