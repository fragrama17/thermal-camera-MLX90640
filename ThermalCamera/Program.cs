namespace ThermalCamera;

public static class Program
{
    public static async Task Main(string[] args)
    {
        var thermalCamera = new ThermalCamera();

        Console.WriteLine($"current refresh rate {thermalCamera.GetRefreshRateToString()}");

        await thermalCamera.SetRefreshRate(ThermalCamera.RefreshRate._4_Hz);
        Console.WriteLine($"new refresh rate {thermalCamera.GetRefreshRateToString()}");

        Console.CancelKeyPress += (_, _) =>
        {
            thermalCamera.Dispose();
            Console.WriteLine("resources released successfully, program exited");
        };

        while (true)
        {
            var frame = await thermalCamera.GetImage();

            PrintImage(frame);
        }
    }

    private static void PrintImage(float[] frame)
    {
        Console.WriteLine("Thermal Image:");
        Console.WriteLine("[");
        for (int i = 0; i < ThermalCamera.TotRows; i++)
        {
            Console.Write("     ");
            for (int j = 0; j < ThermalCamera.TotColumns; j++)
            {
                Console.Write(i * ThermalCamera.TotColumns + j < ThermalCamera.TotPixels - 1
                    ? $"{frame[i * ThermalCamera.TotColumns + j]:F2}, "
                    : $"{frame[i * ThermalCamera.TotColumns + j]:F2}");
            }

            Console.WriteLine("");
        }

        Console.WriteLine("]");
    }
}