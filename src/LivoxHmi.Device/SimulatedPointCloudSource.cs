using System.Diagnostics;
using LivoxHmi.Core;

namespace LivoxHmi.Device;

/// <summary>
/// Deterministic, bounded point-cloud simulator used when MID-360 hardware is unavailable.
/// It intentionally implements the same IPointCloudSource contract as the real device.
/// </summary>
public sealed class SimulatedPointCloudSource : IPointCloudSource
{
    private readonly object _gate = new();
    private PointCloudFrame? _latestFrame;
    private CancellationTokenSource? _cts;
    private Task? _worker;
    private int _pointCount = 24_000;
    private int _fps = 15;
    private SimulationScenario _scenario = SimulationScenario.DemoRoom;
    private long _sequence;
    private long _framesProduced;
    private long _packetsReceived;
    private long _packetsDropped;
    private readonly Stopwatch _clock = Stopwatch.StartNew();

    public string Name => "Simulator";
    public bool IsConnected => _worker is { IsCompleted: false };
    public ulong FramesProduced => (ulong)Volatile.Read(ref _framesProduced);
    public ulong PacketsReceived => (ulong)Volatile.Read(ref _packetsReceived);
    public ulong PacketsDropped => (ulong)Volatile.Read(ref _packetsDropped);
    public uint QueueDepth => _latestFrame is null ? 0u : 1u;

    public int PointCount
    {
        get => Volatile.Read(ref _pointCount);
        set => Volatile.Write(ref _pointCount, Math.Clamp(value, 2_000, 120_000));
    }

    public int Fps
    {
        get => Volatile.Read(ref _fps);
        set => Volatile.Write(ref _fps, Math.Clamp(value, 1, 30));
    }

    public SimulationScenario Scenario
    {
        get => _scenario;
        set => _scenario = value;
    }

    public void Connect(string configuration)
    {
        if (IsConnected) return;
        _cts = new CancellationTokenSource();
        _worker = Task.Run(() => ProduceLoopAsync(_cts.Token));
    }

    public void Disconnect()
    {
        var cts = Interlocked.Exchange(ref _cts, null);
        if (cts is null) return;
        cts.Cancel();
        try { _worker?.Wait(1000); } catch { }
        cts.Dispose();
        _worker = null;
        Interlocked.Exchange(ref _latestFrame, null);
    }

    public bool TryGetFrame(out PointCloudFrame? frame)
    {
        frame = Interlocked.Exchange(ref _latestFrame, null);
        return frame is not null;
    }

    private async Task ProduceLoopAsync(CancellationToken ct)
    {
        var next = Stopwatch.GetTimestamp();
        while (!ct.IsCancellationRequested)
        {
            try
            {
                var fps = Fps;
                var period = Stopwatch.Frequency / Math.Max(1, fps);
                var now = Stopwatch.GetTimestamp();
                if (now >= next)
                {
                    next = now + period;
                    var frame = GenerateFrame();
                    Interlocked.Exchange(ref _latestFrame, frame);
                    Interlocked.Increment(ref _framesProduced);
                    Interlocked.Add(ref _packetsReceived, Math.Max(1, frame.PacketCount));
                }

                await Task.Delay(1, ct).ConfigureAwait(false);
            }
            catch (OperationCanceledException) { break; }
        }
    }

    private PointCloudFrame GenerateFrame()
    {
        var requested = PointCount;
        var points = new Point3D[requested];
        var rng = new Random(unchecked((int)(Environment.TickCount ^ _sequence)));
        var index = 0;

        // 1) Horizontal floor. It is deliberately dense enough to make camera
        // orientation and point-cloud scale immediately obvious.
        var floorCount = Math.Min((int)(requested * 0.55), requested);
        for (int i = 0; i < floorCount; i++)
        {
            var x = (float)(-4.0 + rng.NextDouble() * 8.0);
            var y = (float)(-3.0 + rng.NextDouble() * 6.0);
            var z = Noise(rng, 0.004f);
            points[index++] = new Point3D(x, y, z, 80, 0);
        }

        if (_scenario != SimulationScenario.FloorOnly)
        {
            // 2) Back wall at Y = 3 m.
            var wallCount = Math.Min((int)(requested * 0.18), requested - index);
            for (int i = 0; i < wallCount; i++)
            {
                var x = (float)(-4.0 + rng.NextDouble() * 8.0);
                var z = (float)(0.1 + rng.NextDouble() * 2.8);
                var y = 3.0f + Noise(rng, 0.004f);
                points[index++] = new Point3D(x, y, z, 100, 0);
            }

            // 3) Static boxes/columns.
            index = AddBox(points, index, requested, rng, -2.1f, -0.8f, 0.0f, 1.2f, 0.9f, 1.4f);
            index = AddBox(points, index, requested, rng, 1.4f, 0.7f, 0.0f, 1.0f, 1.0f, 1.8f);
        }

        if (_scenario == SimulationScenario.DemoRoom || _scenario == SimulationScenario.MovingObject)
        {
            // 4) A moving human-sized cluster. This is only a moving-object test cluster;
            // it simply gives us a dynamic object for renderer testing.
            var t = _clock.Elapsed.TotalSeconds;
            var cx = (float)(1.6 * Math.Sin(t * 0.65));
            var cy = (float)(-0.8 + 0.5 * Math.Cos(t * 0.45));
            index = AddHumanCluster(points, index, requested, rng, cx, cy);
        }

        // 5) Optional bounded random noise for robustness testing.
        if (_scenario == SimulationScenario.Stress)
        {
            while (index < requested)
            {
                var x = (float)(-5.0 + rng.NextDouble() * 10.0);
                var y = (float)(-4.0 + rng.NextDouble() * 8.0);
                var z = (float)(-0.1 + rng.NextDouble() * 3.2);
                points[index++] = new Point3D(x, y, z, (byte)rng.Next(20, 180), 1);
            }
        }
        else
        {
            // Fill remaining slots with a small number of additional floor points.
            while (index < requested)
            {
                var x = (float)(-4.0 + rng.NextDouble() * 8.0);
                var y = (float)(-3.0 + rng.NextDouble() * 6.0);
                points[index++] = new Point3D(x, y, Noise(rng, 0.006f), 70, 0);
            }
        }

        return new PointCloudFrame(
            DateTimeOffset.UtcNow,
            0,
            points,
            0,
            Math.Max(1, requested / 120));
    }

    private static int AddBox(Point3D[] points, int index, int max, Random rng,
        float cx, float cy, float z0, float sx, float sy, float sz)
    {
        var target = Math.Min(max, index + 1_500);
        while (index < target)
        {
            var face = rng.Next(6);
            var x = cx + (float)((rng.NextDouble() - 0.5) * sx);
            var y = cy + (float)((rng.NextDouble() - 0.5) * sy);
            var z = z0 + (float)(rng.NextDouble() * sz);
            switch (face)
            {
                case 0: x = cx - sx / 2; break;
                case 1: x = cx + sx / 2; break;
                case 2: y = cy - sy / 2; break;
                case 3: y = cy + sy / 2; break;
                case 4: z = z0; break;
                case 5: z = z0 + sz; break;
            }
            points[index++] = new Point3D(x, y, z + Noise(rng, 0.003f), 120, 0);
        }
        return index;
    }

    private static int AddHumanCluster(Point3D[] points, int index, int max, Random rng,
        float cx, float cy)
    {
        var target = Math.Min(max, index + 4_000);
        while (index < target)
        {
            // Simple vertical ellipsoid: torso/head/shoulder-like cloud.
            var u = rng.NextDouble() * 2 - 1;
            var v = rng.NextDouble() * 2 - 1;
            var w = rng.NextDouble() * 2 - 1;
            if (u * u + v * v + w * w > 1) continue;

            var x = cx + (float)(u * 0.45);
            var y = cy + (float)(v * 0.28);
            var z = (float)(0.15 + (w + 1.0) * 0.5 * 1.65);
            points[index++] = new Point3D(x, y, z, 140, 0);
        }
        return index;
    }

    private static float Noise(Random rng, float amplitude) =>
        (float)((rng.NextDouble() - 0.5) * 2.0 * amplitude);

    public void Dispose() => Disconnect();
}

public enum SimulationScenario
{
    DemoRoom,
    MovingObject,
    FloorOnly,
    Stress
}
