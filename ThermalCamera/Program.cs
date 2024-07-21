using System.Device.I2c;

namespace ThermalCamera;

public static class Program
{
    public static async Task Main(string[] args)
    {
        var thermalCamera = new ThermalCamera();
        Console.WriteLine($"current refresh rate {thermalCamera.GetRefreshRateHz()}");
        Console.CancelKeyPress += (_, _) =>
        {
            thermalCamera.Dispose();
            Console.WriteLine("resources released successfully, program exited");
        };
        while (true)
        {
            // const int statusReady = 0x8000;
            // const int refreshRateHz = 0x800D;
            
            // var device = I2cBus.Create(1).CreateDevice(0x33);

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

            var frame = await thermalCamera.GetImage();
            // var frame = new ushort[834];
            // var status = await thermalCamera.GetFrameData(frame);
            // if (status < 0)
            // {
                // Console.WriteLine($"error while reading data frame ! error code: {status}");
                // continue;
            // }

            PrintFrame(frame);
        }
    }

    // private static void PrintFrame(float[] frame)
    private static void PrintFrame(ushort[] frame)
    {
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
    }


}