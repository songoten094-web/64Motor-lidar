namespace WaveMotionControl.Models;

public sealed class LineConnection
{
    public required int LineNumber { get; init; }
    public string PortName { get; set; } = string.Empty;
    public int BaudRate { get; set; } = 115200;
    public bool IsConnected { get; set; }
}
