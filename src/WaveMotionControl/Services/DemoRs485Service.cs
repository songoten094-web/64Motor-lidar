using WaveMotionControl.Models;
using WaveMotionControl.State;

namespace WaveMotionControl.Services;

/// <summary>
/// Mô phỏng giao tiếp để kiểm tra giao diện. Không gửi dữ liệu ra cổng COM thật.
/// Thay lớp này bằng Em2RsModbusService khi tích hợp phần cứng.
/// </summary>
public sealed partial class DemoRs485Service : IRs485Service, IModeDriverSettingsService
{
    private readonly ApplicationState _state;
    private readonly Dictionary<AxisAddress, DriverModeSettings> _demoSettings = new();
    private CancellationTokenSource? _autoCts;
    private AutoProgram? _activeAutoProgram;
    private readonly object _demoLidarSync = new();
    private readonly Dictionary<int, int?> _demoLidarZones = new();

    public DemoRs485Service(ApplicationState state)
    {
        _state = state;
    }

    public async Task ConnectLineAsync(int line, string portName, int baudRate, CancellationToken cancellationToken = default)
    {
        ValidateLine(line);
        _state.WriteLog(LogLevel.Info, $"Line {line}: mở {portName} tại {baudRate} bps...");
        await Task.Delay(250, cancellationToken);

        var connection = _state.Lines[line - 1];
        connection.PortName = portName;
        connection.BaudRate = baudRate;
        connection.IsConnected = true;

        foreach (var axis in _state.GetAxesForLine(line))
        {
            axis.State = AxisMotionState.Online;
            axis.LastCommand = "CONNECTED";
        }

        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Ok, $"Line {line}: tìm thấy 16/16 driver ({line}.1 → {line}.16).");
    }

    public Task DisconnectLineAsync(int line, CancellationToken cancellationToken = default)
    {
        ValidateLine(line);
        var connection = _state.Lines[line - 1];
        connection.IsConnected = false;

        foreach (var axis in _state.GetAxesForLine(line))
        {
            axis.State = AxisMotionState.Offline;
            axis.VelocityRpm = 0;
            axis.LastCommand = "DISCONNECTED";
        }

        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Warning, $"Line {line}: đã ngắt {connection.PortName}.");
        return Task.CompletedTask;
    }

    public async Task HomeAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default)
    {
        var targets = axes.Distinct().ToArray();
        if (targets.Length == 0) return;

        foreach (var address in targets)
        {
            var axis = _state.GetAxis(address);
            if (!axis.IsOnline)
            {
                _state.WriteLog(LogLevel.Warning, $"Driver {address}: bỏ qua vì đang offline.");
                continue;
            }

            axis.State = AxisMotionState.Homing;
            axis.LastCommand = "HOME 0x6002=0x0020";
        }

        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Info, $"Bắt đầu Homing {targets.Count(a => _state.GetAxis(a).IsOnline)} driver.");

        foreach (var address in targets)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var axis = _state.GetAxis(address);
            if (axis.State != AxisMotionState.Homing) continue;

            await Task.Delay(55, cancellationToken);
            axis.PositionRevolutions = 0;
            axis.VelocityRpm = 0;
            axis.State = AxisMotionState.Homed;
            axis.LastCommand = "HOME_OK";
            _state.NotifyStateChanged();
            _state.WriteLog(LogLevel.Ok, $"Driver {address}: HOME_OK.");
        }
    }

    public Task SetCurrentPositionAsOriginAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default)
    {
        var targets = axes.Distinct().ToArray();
        if (targets.Length == 0) return Task.CompletedTask;

        var offline = targets.Where(a => !_state.GetAxis(a).IsOnline).ToArray();
        if (offline.Length > 0)
        {
            throw new InvalidOperationException(
                $"Không thể lấy gốc: driver offline {string.Join(", ", offline.Select(a => a.DisplayId))}.");
        }

        foreach (var address in targets)
        {
            var axis = _state.GetAxis(address);
            axis.PositionRevolutions = 0;
            axis.VelocityRpm = 0;
            axis.State = AxisMotionState.Homed;
            axis.LastCommand = "CURRENT_POSITION_ORIGIN";
            axis.AlarmText = string.Empty;
        }

        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Ok,
            $"[Mô phỏng AUTO ORIGIN] Đã lấy vị trí hiện tại làm pha 0 cho {targets.Length} driver.");
        return Task.CompletedTask;
    }

    public Task StartJogAsync(AxisAddress axisAddress, JogDirection direction, int speedRpm, int acceleration, int deceleration, CancellationToken cancellationToken = default)
    {
        var axis = RequireOnline(axisAddress);
        axis.State = direction == JogDirection.Forward
            ? AxisMotionState.JoggingForward
            : AxisMotionState.JoggingReverse;
        axis.VelocityRpm = direction == JogDirection.Forward ? speedRpm : -speedRpm;
        axis.LastCommand = $"JOG {(direction == JogDirection.Forward ? "+" : "-")}";

        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Info,
            $"Driver {axisAddress}: {axis.LastCommand}, tốc độ {speedRpm} rpm, Acc {acceleration}, Dec {deceleration}.");
        return Task.CompletedTask;
    }

    public Task StopAxisAsync(AxisAddress axisAddress, CancellationToken cancellationToken = default)
    {
        var axis = _state.GetAxis(axisAddress);
        if (!axis.IsOnline) return Task.CompletedTask;

        axis.VelocityRpm = 0;
        axis.State = axis.PositionRevolutions == 0 ? AxisMotionState.Homed : AxisMotionState.Online;
        axis.LastCommand = "STOP";
        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Ok, $"Driver {axisAddress}: dừng chuyển động.");
        return Task.CompletedTask;
    }

    private bool _isAutoPaused;

    public async Task MoveRelativeRevolutionsAsync(AxisAddress axisAddress, double signedRevolutions, int speedRpm, int pulsesPerRevolution, CancellationToken cancellationToken = default)
    {
        var axis = RequireOnline(axisAddress);
        var rawPulses = Math.Round(signedRevolutions * pulsesPerRevolution);
        var pulses = (long)Math.Clamp(rawPulses, int.MinValue, int.MaxValue);

        axis.State = AxisMotionState.Moving;
        axis.VelocityRpm = Math.Sign(signedRevolutions) * speedRpm;
        axis.LastCommand = $"MOVE REL {pulses} p";
        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Info,
            $"Driver {axisAddress}: chạy tương đối {signedRevolutions:0.###} vòng = {pulses:N0} p tại {speedRpm} rpm.");

        var estimatedMs = Math.Clamp(Math.Abs(signedRevolutions) / Math.Max(1, speedRpm) * 60_000, 400, 5_000);
        await Task.Delay((int)estimatedMs, cancellationToken);

        axis.PositionRevolutions += signedRevolutions;
        axis.VelocityRpm = 0;
        axis.State = AxisMotionState.Online;
        axis.LastCommand = "MOVE_COMPLETE";
        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Ok, $"Driver {axisAddress}: MOVE_COMPLETE, vị trí {axis.PositionRevolutions:0.###} vòng.");
    }

    public Task StartAutoAsync(AutoProgram program, CancellationToken cancellationToken = default)
    {
        _autoCts?.Cancel();
        _autoCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        _isAutoPaused = false;

        var requested = program.Clusters.SelectMany(c => c.Cells)
            .Select(c => c.DriverId)
            .ToArray();
        if (requested.Any(driver => driver is null))
            throw new InvalidOperationException("AUTO demo: còn ô chưa gán Driver ID.");

        var addresses = requested.Select(driver => driver!.Value).Distinct().ToArray();
        var offline = addresses.Where(a => !_state.GetAxis(a).IsOnline).ToArray();
        if (offline.Length > 0)
            throw new InvalidOperationException($"AUTO demo: driver offline {string.Join(", ", offline)}.");

        var noOrigin = addresses.Where(a =>
        {
            var axis = _state.GetAxis(a);
            return axis.State != AxisMotionState.Homed || Math.Abs(axis.PositionRevolutions) > 0.02;
        }).ToArray();
        if (noOrigin.Length > 0)
            throw new InvalidOperationException($"AUTO demo: driver chưa ở pha 0 {string.Join(", ", noOrigin)}.");

        var phaseOffsets = new Dictionary<AxisAddress, double>();
        var speeds = new Dictionary<AxisAddress, double>();
        var driverClusters = program.DriverClusters();

        lock (_demoLidarSync)
        {
            _activeAutoProgram = program;
            _demoLidarZones.Clear();
            foreach (var lidarCluster in program.Clusters.Where(c => c.Effect == AutoEffectType.Lidar))
            {
                _demoLidarZones[lidarCluster.Id] = null;
            }
        }

        foreach (var cluster in program.Clusters)
        {
            if (cluster.Effect == AutoEffectType.Lidar)
            {
                foreach (var cell in cluster.Cells.Where(c => c.DriverId is not null))
                {
                    var address = cell.DriverId!.Value;
                    phaseOffsets[address] = cluster.GetLidarRandomPhase(address);
                    speeds[address] = Math.Max(0.0001, cluster.FrequencyHz);
                }
                continue;
            }

            var layers = cluster.BuildWaveLayers();
            var maxLayerIndex = layers.Count == 0 ? 0 : layers.Max(layer => layer.Index);
            foreach (var layer in layers)
            {
                var rawPhase =
                    (maxLayerIndex - layer.Index) *
                    Math.Max(0, cluster.LayerOffsetRevolutions);
                var phase = rawPhase % 1.0;
                foreach (var address in layer.Drivers)
                {
                    phaseOffsets[address] = phase;
                    speeds[address] = Math.Max(0.0001, cluster.FrequencyHz);
                }
            }
        }

        foreach (var address in addresses)
        {
            var axis = _state.GetAxis(address);
            axis.State = AxisMotionState.Moving;
            axis.LastCommand = "AUTO_16PR_INTERNAL_DEMO";
            axis.PositionRevolutions = phaseOffsets.GetValueOrDefault(address);
            axis.VelocityRpm = (int)Math.Round(speeds.GetValueOrDefault(address, 0.2) * 60.0);
        }

        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Ok,
            $"AUTO 16PR INTERNAL demo: {program.Clusters.Count} cụm, {addresses.Length} driver; " +
            "đặt sẵn phase rồi cùng chạy, không delay layer.");

        var token = _autoCts.Token;
        _ = Task.Run(async () =>
        {
            var clock = System.Diagnostics.Stopwatch.StartNew();
            var lastTicks = clock.ElapsedTicks;
            while (!token.IsCancellationRequested)
            {
                try { await Task.Delay(50, token); }
                catch (OperationCanceledException) { break; }

                var nowTicks = clock.ElapsedTicks;
                var delta = (nowTicks - lastTicks) / (double)System.Diagnostics.Stopwatch.Frequency;
                lastTicks = nowTicks;
                if (_isAutoPaused) continue;

                foreach (var address in addresses)
                {
                    var cluster = driverClusters[address];
                    int? activeLidarZone = null;
                    if (cluster.Effect == AutoEffectType.Lidar)
                    {
                        lock (_demoLidarSync)
                        {
                            _demoLidarZones.TryGetValue(cluster.Id, out activeLidarZone);
                        }
                    }

                    var axis = _state.GetAxis(address);
                    var speedRps = speeds.GetValueOrDefault(address, 0.2);
                    axis.PositionRevolutions += Math.Max(0, delta) * speedRps;
                    axis.VelocityRpm = (int)Math.Round(speedRps * 60.0);
                    axis.LastCommand = cluster.Effect == AutoEffectType.Lidar
                        ? activeLidarZone is int zone
                            ? $"LIDAR_WAVE_Z{zone + 1}_RUNNING"
                            : "LIDAR_RANDOM_RUNNING"
                        : "AUTO_16PR_INTERNAL_RUNNING";
                }
                _state.NotifyStateChanged();
            }
        }, token);

        return Task.CompletedTask;
    }

    public async Task SetLidarZoneAsync(
        int clusterId,
        int? zeroBasedZoneColumn,
        CancellationToken cancellationToken = default)
    {
        AutoProgram program;
        CancellationToken autoToken;
        int? activeLockedZone;
        lock (_demoLidarSync)
        {
            program = _activeAutoProgram
                ?? throw new InvalidOperationException("AUTO demo chưa chạy.");
            autoToken = _autoCts?.Token ?? cancellationToken;
            _demoLidarZones.TryGetValue(clusterId, out activeLockedZone);
        }

        var cluster = program.Clusters.FirstOrDefault(c => c.Id == clusterId)
            ?? throw new InvalidOperationException($"Không tìm thấy Cụm {clusterId}.");
        if (cluster.Effect != AutoEffectType.Lidar)
            throw new InvalidOperationException($"Cụm {clusterId} không dùng hiệu ứng LIDAR.");

        if (zeroBasedZoneColumn is int zone && (zone < 0 || zone >= cluster.Width))
            throw new ArgumentOutOfRangeException(nameof(zeroBasedZoneColumn));

        if (activeLockedZone is int lockedZone)
        {
            _state.WriteLog(LogLevel.Info,
                $"[Mô phỏng LIDAR] Cụm {clusterId}: đang khóa Zone {lockedZone + 1} trong 60 giây; bỏ qua tín hiệu mới.");
            return;
        }

        if (zeroBasedZoneColumn is null)
            return;

        var activeZone = zeroBasedZoneColumn.Value;
        var cells = cluster.Cells.Where(c => c.DriverId is not null).ToArray();

        double NextForwardPhase(double current, double phase)
        {
            var cycle = Math.Floor(current);
            var target = cycle + phase;
            if (target <= current + 0.001) target += 1.0;
            return target;
        }

        lock (_demoLidarSync)
        {
            if (_demoLidarZones.TryGetValue(clusterId, out var raceZone) && raceZone is int)
                return;
            _demoLidarZones[clusterId] = activeZone;
        }

        foreach (var cell in cells)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var address = cell.DriverId!.Value;
            var axis = _state.GetAxis(address);
            var localColumn = cell.Column - cluster.LeftColumn;
            var targetPhase = cluster.GetLidarTargetRevolutions(activeZone, localColumn);
            axis.PositionRevolutions = NextForwardPhase(axis.PositionRevolutions, targetPhase);
            axis.VelocityRpm = (int)Math.Round(cluster.FrequencyHz * 60.0 * 2.0);
            axis.State = AxisMotionState.Moving;
            axis.LastCommand = $"LIDAR_PHASE_2X_Z{activeZone + 1}_{targetPhase:0.###}REV";
        }

        _state.NotifyStateChanged();
        await Task.Delay(80, cancellationToken).ConfigureAwait(false);

        foreach (var cell in cells)
        {
            var axis = _state.GetAxis(cell.DriverId!.Value);
            axis.VelocityRpm = (int)Math.Round(cluster.FrequencyHz * 60.0);
            axis.LastCommand = $"LIDAR_WAVE_Z{activeZone + 1}_60S_RUNNING";
        }
        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Ok,
            $"[Mô phỏng LIDAR] Cụm {clusterId}: Zone {activeZone + 1} khóa; phase @2X, chạy wave 60 giây @1X.");

        _ = RunDemoLidarWaveWindowAsync(program, cluster, activeZone, autoToken);
    }

    private async Task RunDemoLidarWaveWindowAsync(
        AutoProgram program,
        AutoCluster cluster,
        int activeZone,
        CancellationToken cancellationToken)
    {
        try
        {
            await Task.Delay(TimeSpan.FromSeconds(60), cancellationToken).ConfigureAwait(false);
            while (_isAutoPaused)
            {
                await Task.Delay(100, cancellationToken).ConfigureAwait(false);
            }

            lock (_demoLidarSync)
            {
                if (!ReferenceEquals(_activeAutoProgram, program) ||
                    !_demoLidarZones.TryGetValue(cluster.Id, out var zone) ||
                    zone != activeZone)
                {
                    return;
                }
            }

            var cells = cluster.Cells.Where(c => c.DriverId is not null).ToArray();
            foreach (var cell in cells)
            {
                var address = cell.DriverId!.Value;
                var axis = _state.GetAxis(address);
                var randomPhase = cluster.GetLidarRandomPhase(address);
                var cycle = Math.Floor(axis.PositionRevolutions);
                var target = cycle + randomPhase;
                if (target <= axis.PositionRevolutions + 0.001) target += 1.0;
                axis.PositionRevolutions = target;
                axis.VelocityRpm = (int)Math.Round(cluster.FrequencyHz * 60.0);
                axis.State = AxisMotionState.Moving;
                axis.LastCommand = "LIDAR_RANDOM_RUNNING";
            }

            lock (_demoLidarSync)
            {
                _demoLidarZones[cluster.Id] = null;
            }
            _state.NotifyStateChanged();
            _state.WriteLog(LogLevel.Ok,
                $"[Mô phỏng LIDAR] Cụm {cluster.Id}: đủ 60 giây -> trở lại RANDOM.");
        }
        catch (OperationCanceledException)
        {
            // AUTO STOP.
        }
    }

    public Task PauseAutoAsync(bool paused, CancellationToken cancellationToken = default)
    {
        _isAutoPaused = paused;
        _state.WriteLog(LogLevel.Info, paused ? "AUTO PAUSE." : "AUTO RESUME.");
        return Task.CompletedTask;
    }

    public Task StopAllAsync(bool quickStop, CancellationToken cancellationToken = default)
    {
        _autoCts?.Cancel();
        _isAutoPaused = false;
        lock (_demoLidarSync)
        {
            _activeAutoProgram = null;
            _demoLidarZones.Clear();
        }
        foreach (var axis in _state.Axes.Where(a => a.IsOnline))
        {
            axis.VelocityRpm = 0;
            axis.State = Math.Abs(axis.PositionRevolutions) <= 0.02
                ? AxisMotionState.Homed
                : AxisMotionState.Online;
            axis.LastCommand = quickStop ? "QUICK_STOP" : "AUTO_STOP";
        }

        _state.NotifyStateChanged();
        _state.WriteLog(quickStop ? LogLevel.Error : LogLevel.Ok,
            quickStop ? "QUICK STOP gửi tới toàn bộ hệ thống." : "RAMP STOP toàn bộ hệ thống.");
        return Task.CompletedTask;
    }

    private AxisRuntime RequireOnline(AxisAddress address)
    {
        var axis = _state.GetAxis(address);
        if (!axis.IsOnline)
        {
            throw new InvalidOperationException($"Driver {address} đang offline.");
        }
        return axis;
    }

    public Task SaveDriverConfigAsync(IEnumerable<AxisAddress> axes, int diPinIndex, bool activeLowNC, double peakCurrentAmps, int standbyPercent, double homingSpeedRps, double autoSpeedRps, double autoAccRps2, CancellationToken cancellationToken = default)
    {
        var targets = axes.Distinct().ToArray();
        var logicStr = activeLowNC ? "N.C (Low Active)" : "N.O (High Active)";
        _state.WriteLog(LogLevel.Ok, $"[Mô phỏng EEPROM] Đã lưu vĩnh viễn cho {targets.Length} driver: Chân DI{diPinIndex} ({logicStr}), Peak={peakCurrentAmps:0.0}A, HomingSpeed={homingSpeedRps:0.0}v/s, AutoSpeed={autoSpeedRps:0.0}v/s, AutoAcc={autoAccRps2:0}v/s².");
        return Task.CompletedTask;
    }

    public Task ClearDriverConfigAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default)
    {
        var targets = axes.Distinct().ToArray();
        _state.WriteLog(LogLevel.Warning, $"[Mô phỏng EEPROM] Đã xóa toàn bộ cấu hình (Factory Reset) cho {targets.Length} driver về mặc định.");
        return Task.CompletedTask;
    }

    public Task<(int diPinIndex, bool activeLowNC, double peakCurrentAmps, int standbyPercent, double homingSpeedRps, double autoSpeedRps, double autoAccRps2)> ReadDriverConfigAsync(AxisAddress axis, CancellationToken cancellationToken = default)
    {
        _state.WriteLog(LogLevel.Info, $"[Mô phỏng EEPROM] Đọc cấu hình từ Driver {axis}: Chân DI5 (N.O), Peak=3.0A, HomingSpeed=2.0v/s, AutoSpeed=10.0v/s, AutoAcc=500v/s².");
        return Task.FromResult((5, false, 3.0, 50, 2.0, 10.0, 500.0));
    }

    public Task ResetAlarmAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default)
    {
        var targets = axes.Distinct().ToArray();
        foreach (var addr in targets)
        {
            var axis = _state.GetAxis(addr);
            if (axis.State == AxisMotionState.Alarm)
            {
                axis.State = AxisMotionState.Online;
                axis.LastCommand = "ALARM_CLEARED";
            }
        }
        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Ok, $"[Mô phỏng Reset Alarm] Đã gửi lệnh xóa lỗi cho {targets.Length} driver. Đèn chuyển sang xanh lá.");
        return Task.CompletedTask;
    }

    public Task SaveCompleteDriverSettingsAsync(IEnumerable<AxisAddress> axes, int diPinIndex, bool activeLowNC, int standbyPercent, double autoSpeedRps, double autoAccRps2, DriverModeSettings settings, CancellationToken cancellationToken = default)
    {
        var targets = axes.Distinct().ToArray();
        foreach (var addr in targets)
        {
            _demoSettings[addr] = settings;
        }
        _state.WriteLog(LogLevel.Ok, $"[Mô phỏng Setting Complete] Đã lưu hoàn chỉnh cho {targets.Length} driver: DI{diPinIndex}, Standby={standbyPercent}%, Home={settings.HomeCurrentAmps}A, Manual={settings.ManualCurrentAmps}A, Auto={settings.AutoCurrentAmps}A, PPR={settings.PulsesPerRevolution:N0}.");
        return Task.CompletedTask;
    }

    public Task SaveModeDriverSettingsAsync(IEnumerable<AxisAddress> axes, DriverModeSettings settings, CancellationToken cancellationToken = default)
    {
        var targets = axes.Distinct().ToArray();
        foreach (var addr in targets)
        {
            _demoSettings[addr] = settings;
        }
        _state.WriteLog(LogLevel.Ok, $"[Mô phỏng Profile] Đã lưu Profile cho {targets.Length} driver: Home={settings.HomeCurrentAmps}A, Manual={settings.ManualCurrentAmps}A, Auto={settings.AutoCurrentAmps}A, PPR={settings.PulsesPerRevolution:N0}.");
        return Task.CompletedTask;
    }

    public Task<DriverModeSettings> ReadModeDriverSettingsAsync(AxisAddress address, CancellationToken cancellationToken = default)
    {
        if (!_demoSettings.TryGetValue(address, out var settings))
        {
            settings = DriverModeSettings.Default;
        }
        _state.WriteLog(LogLevel.Info, $"[Mô phỏng Profile] Đọc Profile từ Driver {address}: Home={settings.HomeCurrentAmps}A, Manual={settings.ManualCurrentAmps}A, Auto={settings.AutoCurrentAmps}A, PPR={settings.PulsesPerRevolution:N0}.");
        return Task.FromResult(settings);
    }

    public int GetConfiguredPulsesPerRevolution(AxisAddress address)
    {
        return _demoSettings.TryGetValue(address, out var settings) ? settings.PulsesPerRevolution : 10_000;
    }

    private static void ValidateLine(int line)
    {
        if (line is < 1 or > 4)
        {
            throw new ArgumentOutOfRangeException(nameof(line));
        }
    }
}
