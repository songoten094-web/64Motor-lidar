namespace LivoxHmi.Core;

/// <summary>
/// Common boundary for all point-cloud producers.
/// The UI and processing pipeline never depend on the transport implementation.
/// </summary>
public interface IPointCloudSource : IDisposable
{
    string Name { get; }
    bool IsConnected { get; }
    ulong FramesProduced { get; }
    ulong PacketsReceived { get; }
    ulong PacketsDropped { get; }
    uint QueueDepth { get; }

    void Connect(string configuration);
    void Disconnect();
    bool TryGetFrame(out PointCloudFrame? frame);
}
