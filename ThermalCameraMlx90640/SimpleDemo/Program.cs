using ThermalCameraMlx90640;

namespace SimpleDemo;

public static class Program
{
    public static async Task Main(string[] args)
    {
        var thermalCamera = new ThermalCamera();

        Console.WriteLine($"current refresh rate {thermalCamera.GetRefreshRateToString()}");

        await thermalCamera.SetRefreshRate(ThermalCamera.RefreshRate._2_Hz);
        Console.WriteLine($"new refresh rate {thermalCamera.GetRefreshRateToString()}");

        Console.CancelKeyPress += (_, _) =>
        {
            thermalCamera.Dispose();
            Console.WriteLine("resources released successfully, program exited");
        };

        while (true)
        {
            var matrix = await thermalCamera.GetImageAsMatrix();
            PrintMatrix(matrix);
        }
    }

    private static void PrintMatrix(float[,] matrix)
    {
        Console.WriteLine("Thermal Matrix:");
        Console.WriteLine("[");
        for (int i = 0; i < matrix.GetLength(0); i++)
        {
            Console.Write("     ");
            for (int j = 0; j < matrix.GetLength(1); j++)
            {
                Console.Write(i * ThermalCamera.TotColumns + j < ThermalCamera.TotPixels - 1
                    ? $"{matrix[i, j]:F2}, "
                    : $"{matrix[i, j]:F2}");
            }
        
            Console.WriteLine("");
        }
        
        Console.WriteLine("]");
    }
}