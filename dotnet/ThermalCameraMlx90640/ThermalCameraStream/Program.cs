using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using ThermalCameraStream;

var provider = new ThermalCameraProvider();

provider.Spin();

var builder = WebApplication.CreateBuilder(args);

var app = builder.Build();

Console.CancelKeyPress += async (_, _) =>
{
    await app.DisposeAsync();
    Console.WriteLine("Web Server successfully disposed");
    provider.Dispose();
};

app.UseWebSockets(
    new WebSocketOptions { KeepAliveInterval = TimeSpan.FromSeconds(10) }
);

app.MapGet("/",
    async context =>
    {
        await context.Response.Body.WriteAsync(new ReadOnlyMemory<byte>("hello mate ! how is it going ?"u8.ToArray()));
    });

app.MapGet("/stream", async context =>
{
    if (!context.WebSockets.IsWebSocketRequest)
    {
        await context.Response.Body.WriteAsync(
            new ReadOnlyMemory<byte>("mate this is a web-socket endpoint"u8.ToArray())
        );
        return;
    }

    var webSocket = await context.WebSockets.AcceptWebSocketAsync();


    while (webSocket.State == WebSocketState.Open)
    {
        try
        {
            var json = JsonSerializer.Serialize(new { thermalFrame = provider.CurrentFrame });

            var buffer = new ArraySegment<byte>(Encoding.UTF8.GetBytes(json));

            await webSocket.SendAsync(buffer, WebSocketMessageType.Text, true, CancellationToken.None);
        }
        catch (Exception e)
        {
            Console.WriteLine(e.Message);
            Console.WriteLine("failed to retrieve thermal frame, trying again...");
        }
    }

    Console.WriteLine("client disconnected from thermal camera ws stream");
});


app.Run("http://0.0.0.0:8080");