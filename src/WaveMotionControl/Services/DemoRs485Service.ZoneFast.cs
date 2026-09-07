using WaveMotionControl.Models;
namespace WaveMotionControl.Services;

/// <summary>Demo-only implementation of the integration API.</summary>
public sealed partial class DemoRs485Service
{
    private readonly HashSet<(int ClusterId, int ZoneColumn)> _demoFastZones = new();

    public async Task TriggerZoneFastAsync(
        int clusterId,
        int zeroBasedZoneColumn,
        double speedMultiplier = 2.0,
        TimeSpan? duration = null,
        CancellationToken cancellationToken = default)
    {
        AutoProgram program;
        lock (_demoLidarSync)
        {
            program = _activeAutoProgram ?? throw new InvalidOperationException("AUTO demo chưa chạy.");
        }

        var cluster = program.Clusters.FirstOrDefault(c => c.Id == clusterId)
            ?? throw new InvalidOperationException($"Không tìm thấy Cụm {clusterId}.");
        if (zeroBasedZoneColumn < 0 || zeroBasedZoneColumn >= cluster.Width)
            throw new ArgumentOutOfRangeException(nameof(zeroBasedZoneColumn));

        var key = (clusterId, zeroBasedZoneColumn);
        lock (_demoFastZones)
        {
            if (!_demoFastZones.Add(key)) return;
        }

        var axes = cluster.Cells
            .Where(c => c.DriverId is not null && c.Column - cluster.LeftColumn == zeroBasedZoneColumn)
            .Select(c => c.DriverId!.Value)
            .Distinct()
            .ToArray();

        foreach (var address in axes)
        {
            var axis = _state.GetAxis(address);
            axis.VelocityRpm = Math.Max(1, (int)Math.Round(axis.VelocityRpm * speedMultiplier));
            axis.LastCommand = $"ZONE_FAST_Z{zeroBasedZoneColumn + 1}";
        }
        _state.NotifyStateChanged();

        try
        {
            await Task.Delay(duration ?? TimeSpan.FromSeconds(30), cancellationToken).ConfigureAwait(false);
            foreach (var address in axes)
            {
                var axis = _state.GetAxis(address);
                axis.VelocityRpm = Math.Max(1, (int)Math.Round(cluster.FrequencyHz * 60.0));
                axis.LastCommand = "AUTO_16PR_INTERNAL_RUNNING";
            }
            _state.NotifyStateChanged();
        }
        finally
        {
            lock (_demoFastZones) _demoFastZones.Remove(key);
        }
    }
}
