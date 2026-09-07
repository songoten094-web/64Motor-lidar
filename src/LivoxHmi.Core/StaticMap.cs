using System.Numerics;
using System.Threading.Channels;

namespace LivoxHmi.Core;

public sealed class StaticMapSettings
{
    public double VoxelSizeMeters { get; set; } = 0.012; // V2.6: thin high-sensitivity baseline
    public int MinHitsPerVoxel { get; set; } = 8;
    public double MaxStdDevMeters { get; set; } = 0.020;
    public double MinRangeMeters { get; set; } = 0.20;
    public double MaxRangeMeters { get; set; } = 30.0;
    public bool RejectNonNormalLivoxTags { get; set; } = true;
    public int MaxVoxels { get; set; } = 1_500_000;
}

public sealed class StaticMapDefinition
{
    public string Frame { get; set; } = "MID360_SENSOR";
    public string FilePath { get; set; } = "projects/Machine_A.staticmap.bin";
    public bool Locked { get; set; }
    public int PointCount { get; set; }
    public int SourceFrames { get; set; }
    public double VoxelSizeMeters { get; set; } = 0.012;
    public DateTimeOffset? CreatedAtUtc { get; set; }
}

public sealed record StaticMapPointStats(Point3D Point, int FrameHits, double StdDevMeters, double OccupancyRatio, double Reliability);

public sealed record StaticMapSnapshot(
    IReadOnlyList<Point3D> Points,
    int SourceFrames,
    int CandidateVoxels,
    int StableVoxels,
    double VoxelSizeMeters,
    DateTimeOffset CreatedAtUtc,
    IReadOnlyList<StaticMapPointStats>? Statistics = null);

/// <summary>
/// Bounded fixed-sensor voxel accumulator. Geometry is ALWAYS raw MID-360 Sensor XYZ.
/// No IMU/view transform, ICP, SLAM pose or scale is applied here.
/// </summary>
public sealed class StaticMapBuilder
{
    private readonly object _gate = new();
    private readonly Dictionary<VoxelKey, VoxelStats> _voxels = new();
    private StaticMapSettings _settings = new();
    private int _frames;
    private long _acceptedPoints;
    private long _droppedFrames;
    private Channel<PointCloudFrame>? _channel;
    private CancellationTokenSource? _workerCts;
    private Task? _workerTask;
    private volatile bool _isBuilding;

    public bool IsBuilding => _isBuilding;

    public void Start(StaticMapSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        if (settings.VoxelSizeMeters <= 0) throw new ArgumentOutOfRangeException(nameof(settings.VoxelSizeMeters));
        Cancel();
        lock (_gate)
        {
            _settings = settings;
            _voxels.Clear();
            _frames = 0;
            _acceptedPoints = 0;
            _droppedFrames = 0;
            _channel = Channel.CreateBounded<PointCloudFrame>(new BoundedChannelOptions(2)
            {
                SingleReader = true,
                SingleWriter = true,
                FullMode = BoundedChannelFullMode.DropOldest
            });
            _workerCts = new CancellationTokenSource();
            _isBuilding = true;
            _workerTask = Task.Run(() => WorkerLoopAsync(_channel.Reader, _workerCts.Token));
        }
    }

    /// <summary>Non-blocking acquisition handoff. If mapping falls behind, old map frames are dropped rather than stalling LiDAR acquisition.</summary>
    public void AddFrame(PointCloudFrame frame)
    {
        if (!_isBuilding) return;
        var channel = _channel;
        if (channel is null || !channel.Writer.TryWrite(frame))
            Interlocked.Increment(ref _droppedFrames);
    }

    public void Cancel()
    {
        _isBuilding = false;
        try { _channel?.Writer.TryComplete(); } catch { }
        try { _workerCts?.Cancel(); } catch { }
        lock (_gate)
        {
            _channel = null;
            _workerCts?.Dispose();
            _workerCts = null;
            _workerTask = null;
            _voxels.Clear();
            _frames = 0;
            _acceptedPoints = 0;
            _droppedFrames = 0;
        }
    }

    private async Task WorkerLoopAsync(ChannelReader<PointCloudFrame> reader, CancellationToken ct)
    {
        try
        {
            await foreach (var frame in reader.ReadAllAsync(ct).ConfigureAwait(false))
                ProcessFrame(frame);
        }
        catch (OperationCanceledException) { }
    }

    private void ProcessFrame(PointCloudFrame frame)
    {
        lock (_gate)
        {
            if (!_isBuilding && _channel is null) return;
            _frames++;
            var s = _settings;
            var voxel = s.VoxelSizeMeters;
            // Mapping is intentionally decimated independently of live rendering/acquisition.
            // 40k samples/frame is enough for a fixed sensor while keeping CPU bounded.
            var stride = Math.Max(1, (int)Math.Ceiling(frame.Points.Count / 40_000.0));

            for (var i = 0; i < frame.Points.Count; i += stride)
            {
                var p = frame.Points[i];
                if (!IsFinite(p)) continue;
                var range = Math.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
                if (range < s.MinRangeMeters || range > s.MaxRangeMeters) continue;
                if (s.RejectNonNormalLivoxTags && !LivoxTagDecoder.Decode(p.Tag).IsNormal) continue;

                var key = new VoxelKey(
                    (int)Math.Floor(p.X / voxel),
                    (int)Math.Floor(p.Y / voxel),
                    (int)Math.Floor(p.Z / voxel));

                if (!_voxels.TryGetValue(key, out var stats))
                {
                    if (_voxels.Count >= s.MaxVoxels) continue;
                    stats = new VoxelStats();
                    _voxels.Add(key, stats);
                }
                stats.Add(p, _frames);
                _acceptedPoints++;
            }
        }
    }

    public StaticMapProgress GetProgress()
    {
        lock (_gate)
        {
            var stable = 0;
            foreach (var v in _voxels.Values)
                if (IsStable(v, _settings)) stable++;
            return new StaticMapProgress(_isBuilding, _frames, _acceptedPoints, _voxels.Count, stable,
                Interlocked.Read(ref _droppedFrames));
        }
    }

    public async Task<StaticMapSnapshot> FinalizeMapAsync(CancellationToken ct = default)
    {
        _isBuilding = false;
        var channel = _channel;
        var worker = _workerTask;
        if (channel is not null) channel.Writer.TryComplete();
        if (worker is not null)
        {
            try { await worker.WaitAsync(ct).ConfigureAwait(false); }
            catch (OperationCanceledException) { throw; }
        }

        lock (_gate)
        {
            var result = new List<Point3D>(_voxels.Count);
            var statistics = new List<StaticMapPointStats>(_voxels.Count);
            var stable = 0;
            foreach (var v in _voxels.Values)
            {
                if (!IsStable(v, _settings)) continue;
                stable++;
                var mp = v.MeanPoint();
                result.Add(mp);
                var occ = _frames > 0 ? Math.Clamp(v.FrameHits / (double)_frames, 0.0, 1.0) : 0.0;
                // Reliability rewards repeatability and a thin return distribution.
                var sigmaScore = Math.Clamp(1.0 - v.StdDevMeters / Math.Max(0.001, _settings.MaxStdDevMeters), 0.0, 1.0);
                var reliability = Math.Clamp(0.65 * occ + 0.35 * sigmaScore, 0.0, 1.0);
                statistics.Add(new StaticMapPointStats(mp, v.FrameHits, v.StdDevMeters, occ, reliability));
            }
            _channel = null;
            _workerTask = null;
            _workerCts?.Dispose();
            _workerCts = null;
            return new StaticMapSnapshot(result, _frames, _voxels.Count, stable,
                _settings.VoxelSizeMeters, DateTimeOffset.UtcNow, statistics);
        }
    }

    private static bool IsStable(VoxelStats v, StaticMapSettings s)
    {
        if (v.FrameHits < s.MinHitsPerVoxel) return false;
        return v.StdDevMeters <= s.MaxStdDevMeters;
    }

    private static bool IsFinite(Point3D p) =>
        float.IsFinite(p.X) && float.IsFinite(p.Y) && float.IsFinite(p.Z);

    private readonly record struct VoxelKey(int X, int Y, int Z);

    private sealed class VoxelStats
    {
        public int Count { get; private set; }
        public int FrameHits { get; private set; }
        private int _lastFrame = -1;
        private double _sx, _sy, _sz, _sxx, _syy, _szz;
        private double _sr;
        private long _stag;

        public double StdDevMeters
        {
            get
            {
                if (Count <= 1) return 0;
                var vx = Math.Max(0, _sxx / Count - Math.Pow(_sx / Count, 2));
                var vy = Math.Max(0, _syy / Count - Math.Pow(_sy / Count, 2));
                var vz = Math.Max(0, _szz / Count - Math.Pow(_sz / Count, 2));
                return Math.Sqrt(vx + vy + vz);
            }
        }

        public void Add(Point3D p, int frameIndex)
        {
            Count++;
            if (_lastFrame != frameIndex)
            {
                _lastFrame = frameIndex;
                FrameHits++;
            }
            _sx += p.X; _sy += p.Y; _sz += p.Z;
            _sxx += p.X * p.X; _syy += p.Y * p.Y; _szz += p.Z * p.Z;
            _sr += p.Reflectivity;
            _stag += p.Tag;
        }

        public Point3D MeanPoint() => new(
            (float)(_sx / Count), (float)(_sy / Count), (float)(_sz / Count),
            (byte)Math.Clamp((int)Math.Round(_sr / Count), 0, 255),
            (byte)Math.Clamp((int)Math.Round((double)_stag / Count), 0, 255));
    }
}

public readonly record struct StaticMapProgress(
    bool IsBuilding,
    int Frames,
    long AcceptedPoints,
    int CandidateVoxels,
    int StableVoxels,
    long DroppedFrames);

public sealed class StaticMapStore
{
    private const int Version = 2;
    private const int Magic = 0x50414D4C; // 'LMAP' little-endian

    public async Task SaveAsync(StaticMapSnapshot map, string path, CancellationToken ct = default)
    {
        var fullPath = Path.GetFullPath(path);
        Directory.CreateDirectory(Path.GetDirectoryName(fullPath)!);
        var temp = fullPath + ".tmp";

        await using (var fs = new FileStream(temp, FileMode.Create, FileAccess.Write, FileShare.None,
            128 * 1024, FileOptions.Asynchronous | FileOptions.WriteThrough))
        await using (var bw = new BinaryWriterStream(fs))
        {
            bw.Write(Magic); bw.Write(Version);
            bw.Write(map.VoxelSizeMeters); bw.Write(map.SourceFrames);
            bw.Write(map.CreatedAtUtc.UtcDateTime.Ticks); bw.Write(map.Points.Count);
            for (var i = 0; i < map.Points.Count; i++)
            {
                ct.ThrowIfCancellationRequested();
                var p = map.Points[i];
                bw.Write(p.X); bw.Write(p.Y); bw.Write(p.Z);
                bw.Write(p.Reflectivity); bw.Write(p.Tag);
                var st = map.Statistics is not null && i < map.Statistics.Count ? map.Statistics[i] : null;
                bw.Write(st?.FrameHits ?? 0);
                bw.Write(st?.StdDevMeters ?? 0.020);
                bw.Write(st?.OccupancyRatio ?? 0.5);
                bw.Write(st?.Reliability ?? 0.5);
            }
            await fs.FlushAsync(ct);
        }
        File.Move(temp, fullPath, true);
    }

    public async Task<StaticMapSnapshot> LoadAsync(string path, CancellationToken ct = default)
    {
        var fullPath = Path.GetFullPath(path);
        await using var fs = new FileStream(fullPath, FileMode.Open, FileAccess.Read, FileShare.Read,
            128 * 1024, FileOptions.Asynchronous | FileOptions.SequentialScan);
        using var br = new BinaryReader(fs);
        if (br.ReadInt32() != Magic) throw new InvalidDataException("Static map magic is invalid.");
        var version = br.ReadInt32();
        if (version is not (1 or 2)) throw new InvalidDataException("Static map version is unsupported.");
        var voxel = br.ReadDouble();
        var frames = br.ReadInt32();
        var ticks = br.ReadInt64();
        var count = br.ReadInt32();
        if (count < 0 || count > 5_000_000) throw new InvalidDataException("Static map point count is invalid.");
        var points = new Point3D[count];
        var stats = new StaticMapPointStats[count];
        for (var i = 0; i < count; i++)
        {
            ct.ThrowIfCancellationRequested();
            var p = new Point3D(br.ReadSingle(), br.ReadSingle(), br.ReadSingle(), br.ReadByte(), br.ReadByte());
            points[i] = p;
            if (version >= 2)
                stats[i] = new StaticMapPointStats(p, br.ReadInt32(), br.ReadDouble(), br.ReadDouble(), br.ReadDouble());
            else
                stats[i] = new StaticMapPointStats(p, 0, 0.020, 0.5, 0.5);
        }
        return new StaticMapSnapshot(points, frames, count, count, voxel, new DateTimeOffset(new DateTime(ticks, DateTimeKind.Utc)), stats);
    }

    private sealed class BinaryWriterStream : IAsyncDisposable
    {
        private readonly BinaryWriter _writer;
        public BinaryWriterStream(Stream stream) => _writer = new BinaryWriter(stream, System.Text.Encoding.UTF8, leaveOpen: true);
        public void Write(int v) => _writer.Write(v);
        public void Write(long v) => _writer.Write(v);
        public void Write(double v) => _writer.Write(v);
        public void Write(float v) => _writer.Write(v);
        public void Write(byte v) => _writer.Write(v);
        public ValueTask DisposeAsync() { _writer.Flush(); _writer.Dispose(); return ValueTask.CompletedTask; }
    }
}
