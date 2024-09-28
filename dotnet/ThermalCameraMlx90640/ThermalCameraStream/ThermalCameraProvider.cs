using ThermalCameraMlx90640;

namespace ThermalCameraStream;

public class ThermalCameraProvider: IDisposable
{
    private readonly ThermalCamera _thermalCamera;
    private readonly CancellationTokenSource _cancellationTokenSource;
    private readonly CancellationToken _cancellationToken;
    private float[][] _currentFrame = [];
    public float[][] CurrentFrame
    {
        get => Interlocked.Exchange(ref _currentFrame, _currentFrame);
        private set => Interlocked.Exchange(ref _currentFrame, value);
    }

    public ThermalCameraProvider()
    {
        _thermalCamera = new ThermalCamera();
        _cancellationTokenSource = new CancellationTokenSource();
        _cancellationToken = _cancellationTokenSource.Token;
    }
    
    public async Task Spin()
    {
        await _thermalCamera.SetRefreshRate(ThermalCamera.RefreshRate._4Hz);

        Task.Run(async () =>
        {
            Console.WriteLine("Thermal Camera thread started");
            while (!_cancellationToken.IsCancellationRequested)
            {
                try
                {
                    var frame = await _thermalCamera.GetImageAsMatrix();
                    CurrentFrame = ConvertToJaggedArray(frame);
                }
                catch (IOException e)
                {
                    Console.WriteLine("failed to retrieve frame");
                    Console.WriteLine(e.Message);
                }
            }
            Console.WriteLine("Thermal Camera thread canceled");
        });
    }

    private static float[][] ConvertToJaggedArray(float[,] matrix)
    {
        int rows = matrix.GetLength(0);
        int cols = matrix.GetLength(1);
        float[][] jaggedArray = new float[rows][];

        for (int i = 0; i < rows; i++)
        {
            jaggedArray[i] = new float[cols];
            for (int j = 0; j < cols; j++)
            {
                jaggedArray[i][j] = matrix[i, j];
            }
        }

        return jaggedArray;
    }

    public void Dispose()
    {
        _thermalCamera.Dispose();
        Console.WriteLine("Thermal Camera successfully disposed");
        _cancellationTokenSource.Cancel();
        _cancellationTokenSource.Dispose();
    }
}