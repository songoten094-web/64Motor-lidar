using WaveMotionControl.Models;

namespace WaveMotionControl.Services;

/// <summary>
/// Integration-only extension for a rising-edge LIDAR Zone event.
/// The existing RS485/Modbus transport, register map, line locks, polling,
/// 16PR writer and AUTO lifecycle remain unchanged and are reused here.
/// </summary>
public sealed partial class Em2RsModbusService
{
    private readonly object _zoneFastSync = new();
    private readonly HashSet<(int ClusterId, int ZoneColumn)> _activeZoneFast = new();
    private readonly SemaphoreSlim _zoneFastTransitionLock = new(1, 1);

    public async Task TriggerZoneFastAsync(
        int clusterId,
        int zeroBasedZoneColumn,
        double speedMultiplier = 2.0,
        TimeSpan? duration = null,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();

        var fastDuration = duration ?? TimeSpan.FromSeconds(30);
        if (fastDuration <= TimeSpan.Zero)
            throw new ArgumentOutOfRangeException(nameof(duration));
        if (!double.IsFinite(speedMultiplier) || speedMultiplier <= 1.0)
            throw new ArgumentOutOfRangeException(nameof(speedMultiplier), "FAST multiplier must be > 1.0.");

        AutoProgram program;
        AutoCluster cluster;
        AutoAxisProfile[] normalProfiles;
        CancellationToken autoToken;

        lock (_autoSync)
        {
            program = _activeAutoProgram
                ?? throw new InvalidOperationException("AUTO chưa chạy.");
            cluster = program.Clusters.FirstOrDefault(c => c.Id == clusterId)
                ?? throw new InvalidOperationException($"Không tìm thấy Cụm {clusterId} trong AUTO đang chạy.");

            if (_autoPaused)
                throw new InvalidOperationException("AUTO đang PAUSE. Hãy RESUME trước khi kích Zone FAST.");
            if (_autoCts is null)
                throw new InvalidOperationException("AUTO lifecycle không còn hoạt động.");
            if (zeroBasedZoneColumn < 0 || zeroBasedZoneColumn >= cluster.Width)
                throw new ArgumentOutOfRangeException(nameof(zeroBasedZoneColumn),
                    $"Zone phải từ 1 đến {cluster.Width}.");

            autoToken = _autoCts.Token;
            normalProfiles = _activeAutoProfiles
                .Where(p => p.ClusterId == clusterId && p.LocalColumn == zeroBasedZoneColumn)
                .OrderBy(p => p.Address.Line)
                .ThenBy(p => p.Address.SlaveId)
                .ToArray();
        }

        if (normalProfiles.Length == 0)
            throw new InvalidOperationException(
                $"Cụm {clusterId}, Zone {zeroBasedZoneColumn + 1}: không có motor được gán.");

        var key = (clusterId, zeroBasedZoneColumn);
        lock (_zoneFastSync)
        {
            // Rising-edge semantics: while this Zone is already in its 30 s window,
            // repeated requests are ignored and do not restart the timer.
            if (!_activeZoneFast.Add(key))
            {
                _state.WriteLog(LogLevel.Info,
                    $"[ZONE FAST] Cụm {clusterId} / Z{zeroBasedZoneColumn + 1}: đang FAST, bỏ qua trigger lặp.");
                return;
            }
        }

        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, autoToken);
        try
        {
            await SetZoneProfilesSpeedAsync(
                normalProfiles,
                speedMultiplier,
                $"ZONE_FAST_Z{zeroBasedZoneColumn + 1}",
                linked.Token).ConfigureAwait(false);

            _state.WriteLog(LogLevel.Ok,
                $"[ZONE FAST] Cụm {clusterId} / Z{zeroBasedZoneColumn + 1}: " +
                $"{normalProfiles.Length} motor chạy {speedMultiplier:0.##}X trong {fastDuration.TotalSeconds:0.#} s.");
        }
        catch
        {
            lock (_zoneFastSync) _activeZoneFast.Remove(key);
            throw;
        }

        // Do not hold the caller/detection thread for 30 seconds.
        _ = RestoreZoneSpeedAfterDelayAsync(
            key,
            normalProfiles,
            fastDuration,
            autoToken);
    }

    private async Task RestoreZoneSpeedAfterDelayAsync(
        (int ClusterId, int ZoneColumn) key,
        AutoAxisProfile[] normalProfiles,
        TimeSpan duration,
        CancellationToken autoToken)
    {
        try
        {
            await Task.Delay(duration, autoToken).ConfigureAwait(false);

            // If AUTO is paused when the window ends, do not restart motors under PAUSE.
            while (_autoPaused)
            {
                autoToken.ThrowIfCancellationRequested();
                await Task.Delay(100, autoToken).ConfigureAwait(false);
            }

            await SetZoneProfilesSpeedAsync(
                normalProfiles,
                1.0,
                $"ZONE_NORMAL_Z{key.ZoneColumn + 1}",
                autoToken).ConfigureAwait(false);

            _state.WriteLog(LogLevel.Ok,
                $"[ZONE FAST] Cụm {key.ClusterId} / Z{key.ZoneColumn + 1}: hết 30 s, trở về tốc độ AUTO.");
        }
        catch (OperationCanceledException) when (autoToken.IsCancellationRequested)
        {
            // AUTO stopped: no restore is needed; StopAll/Auto lifecycle owns the motors.
        }
        catch (Exception ex)
        {
            _state.WriteLog(LogLevel.Error,
                $"[ZONE FAST] Cụm {key.ClusterId} / Z{key.ZoneColumn + 1}: restore lỗi: {ex.Message}");
        }
        finally
        {
            lock (_zoneFastSync) _activeZoneFast.Remove(key);
        }
    }

    private async Task SetZoneProfilesSpeedAsync(
        AutoAxisProfile[] normalProfiles,
        double multiplier,
        string commandLabel,
        CancellationToken cancellationToken)
    {
        if (normalProfiles.Length == 0) return;

        await _zoneFastTransitionLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            var pausedLines = await PausePollingForTargetsAsync(
                normalProfiles.Select(p => p.Address)).ConfigureAwait(false);
            try
            {
                await QuickStopAutoProfilesAsync(normalProfiles, cancellationToken).ConfigureAwait(false);

                var programmedProfiles = normalProfiles
                    .Select(profile => multiplier == 1.0
                        ? profile
                        : profile with
                        {
                            SpeedRpm = checked((ushort)Math.Clamp(
                                (int)Math.Round(profile.SpeedRpm * multiplier), 1, 5000))
                        })
                    .ToArray();

                // Reuse the original 16PR writer. No new Modbus protocol/register path is introduced.
                await Task.WhenAll(
                    programmedProfiles
                        .GroupBy(p => p.Address.Line)
                        .Select(async group =>
                        {
                            foreach (var profile in group.OrderBy(p => p.Address.SlaveId))
                            {
                                cancellationToken.ThrowIfCancellationRequested();
                                await WriteInternal16PrLoopAsync(profile, cancellationToken).ConfigureAwait(false);
                            }
                        })).ConfigureAwait(false);

                await TriggerAutoProfilesAsync(programmedProfiles, cancellationToken).ConfigureAwait(false);

                foreach (var profile in programmedProfiles)
                {
                    var axis = _state.GetAxis(profile.Address);
                    axis.State = AxisMotionState.Moving;
                    axis.VelocityRpm = profile.SpeedRpm;
                    axis.LastCommand = commandLabel;
                    axis.AlarmText = string.Empty;
                }
                _state.NotifyStateChanged();
            }
            finally
            {
                ResumePollingLines(pausedLines);
            }
        }
        finally
        {
            _zoneFastTransitionLock.Release();
        }
    }
}
