using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Text.Json;
using WaveMotionControl.Models;
using WaveMotionControl.State;

namespace WaveMotionControl.Services;

/// <summary>
/// Giao tiếp Modbus RTU thật cho tối đa 4 tuyến RS485, mỗi tuyến 16 driver EM2RS.
///
/// Các thanh ghi trong lớp này bám theo EM2RS Series User Manual V1.5:
/// - DI5/ORG:                0x014D = 0x0027 (N.O.) hoặc 0x00A7 (N.C.)
/// - Dòng Peak:              0x0191, đơn vị 0.1 A
/// - Dòng giữ:               0x01D3, đơn vị %
/// - Homing:                 0x600A, 0x600F..0x6012, kích hoạt 0x6002=0x0020
/// - PR0/AUTO:               0x6200..0x6207
/// - Quick stop:             0x6002=0x0040
/// - Trạng thái chuyển động: 0x1003
/// - Alarm hiện tại:         0x2203
/// - Lưu tham số:            0x1801=0x2211
/// - Lưu mapping I/O:        0x1801=0x2244
/// </summary>
public sealed partial class Em2RsModbusService : IRs485Service, IModeDriverSettingsService, IUniformSliderMotionService, IDisposable
{
    private const ushort ForcedEnableRegister = 0x000F;
    private const ushort MotionStatusRegister = 0x1003;
    private const ushort AlarmRegister = 0x2203;
    private const ushort DigitalInputStatusRegister = 0x0179;
    private const ushort SaveControlRegister = 0x1801;
    private const ushort SaveStatusRegister = 0x1901;

    private const ushort PeakCurrentRegister = 0x0191;
    private const ushort StandbyCurrentRegister = 0x01D3;

    private const ushort HomeModeRegister = 0x600A;
    private const ushort HomeFastSpeedRegister = 0x600F;
    private const ushort HomeSlowSpeedRegister = 0x6010;
    private const ushort HomeAccelerationRegister = 0x6011;
    private const ushort HomeDecelerationRegister = 0x6012;
    private const ushort PrControlRegister = 0x6002;

    private const ushort JogSpeedRegister = 0x6027;
    private const ushort JogAccelerationRegister = 0x6028;
    private const ushort JogDecelerationRegister = 0x6029;

    private const ushort ActualPositionRegister = 0x602C;
    private const ushort FeedbackVelocityRegister = 0x1046;

    private const ushort Pr0ModeRegister = 0x6200;
    private const ushort Pr0PositionHighRegister = 0x6201;
    private const ushort Pr0PositionLowRegister = 0x6202;
    private const ushort Pr0SpeedRegister = 0x6203;
    private const ushort Pr0AccelerationRegister = 0x6204;
    private const ushort Pr0DecelerationRegister = 0x6205;
    private const ushort Pr0PauseRegister = 0x6206;
    private const ushort Pr0TriggerRegister = 0x6207;

    private const ushort CommandHome = 0x0020;
    private const ushort CommandSetCurrentPointZero = 0x0021;
    private const ushort CommandQuickStop = 0x0040;
    private const ushort CommandTriggerPr0 = 0x0010;
    private const ushort CommandSaveParameters = 0x2211;
    private const ushort CommandSaveMappings = 0x2244;
    private const ushort CommandResetCurrentAlarm = 0x1111;
    private const ushort CommandResetParametersKeepMotor = 0x2222;

    private const ushort DiFunctionInvalid = 0x0000;
    private const ushort DiFunctionOrgNo = 0x0027;
    private const ushort DiFunctionOrgNc = 0x00A7;

    private const ushort Pr0AbsoluteInterruptMode = 0x0011;
    private const ushort Pr0RelativeMode = 0x0041;
    private const ushort PrPathOverlapBit = 0x0020;

    private const int DefaultPulsesPerRevolution = 10_000;
    private const double MinimumPeakCurrentAmps = 0.5;
    private const double MaximumPeakCurrentAmps = 6.0;
    private const int MaximumAutoUpdateIntervalMs = 120;
    private const int PollDelayMs = 250;

    // Một số USB-RS485 và EM2RS cần thời gian quay vòng lâu hơn khi ghi nhiều
    // tham số hoặc khi driver đang bận ghi EEPROM. 350 ms quá sát và dễ sinh
    // timeout giả, đặc biệt ở bước đọc trạng thái lưu 0x1901.
    private const int ModbusTimeoutMs = 1500;
    private const int ModbusRequestRetryCount = 5;
    private const int ModbusRetryDelayMs = 60;
    private const int InterCommandDelayMs = 15;
    private const int FrameTurnaroundDelayMs = 4;
    private const int SaveStatusPollDelayMs = 180;
    private const int HomeMonitorPollIntervalMs = 250;
    private const int HomeMonitorMaxConsecutiveCommunicationFailures = 10;

    // USB-RS485 trên Windows có thể bị reset bởi nhiễu, tiết kiệm điện USB hoặc
    // rút/cắm lại thiết bị. Khi SerialPort.IsOpen chuyển về false, service sẽ
    // tự mở lại đúng COM/baud đã lưu thay vì bắt người vận hành Connect lại.
    private const int PortReconnectAttempts = 3;
    private const int PortReconnectDelayMs = 250;
    private const int PortReconnectCooldownMs = 1000;
    private const int PortOpenStabilizationMs = 150;
    private const int PollConsecutiveSlaveFailuresBeforePortRecycle = 4;

    // Index 0 không sử dụng để có thể truy cập trực tiếp bằng số DI.
    private static readonly ushort[] DigitalInputFunctionRegisters =
    {
        0x0000,
        0x0145, // DI1 / SI1
        0x0147, // DI2 / SI2
        0x0149, // DI3 / SI3
        0x014B, // DI4 / SI4
        0x014D, // DI5 / SI5
        0x014F, // DI6 / SI6
        0x0151  // DI7 / SI7
    };

    private readonly ApplicationState _state;

    // Nhiều line có thể Connect/Disconnect/Poll song song. Dictionary thường
    // không an toàn khi đọc/ghi từ nhiều Task, nên dùng ConcurrentDictionary.
    private readonly ConcurrentDictionary<int, SerialPort> _ports = new();
    private readonly ConcurrentDictionary<int, LineSerialSettings> _lineSerialSettings = new();
    private readonly ConcurrentDictionary<int, DateTime> _lastReconnectAttemptUtc = new();
    private readonly ConcurrentDictionary<int, int> _lineReconnectFailures = new();

    private readonly Dictionary<int, SemaphoreSlim> _lineLocks = new();
    private readonly Dictionary<int, CancellationTokenSource> _pollCts = new();
    private readonly Dictionary<int, Task> _pollTasks = new();
    private readonly Dictionary<AxisAddress, int> _pollFailures = new();
    private readonly object _configSync = new();
    private readonly Dictionary<AxisAddress, DriverModeSettings> _axisModeSettings = new();
    private readonly SemaphoreSlim _modeSettingsFileLock = new(1, 1);
    private readonly string _modeSettingsFilePath;

    private readonly object _jogSync = new();
    private readonly Dictionary<AxisAddress, CancellationTokenSource> _jogCts = new();
    private readonly Dictionary<AxisAddress, Task> _jogTasks = new();

    private readonly object _autoSync = new();
    private CancellationTokenSource? _autoCts;
    private Task? _autoTask;
    private volatile bool _autoPaused;
    private AutoProgram? _activeAutoProgram;
    private List<AutoAxisProfile> _activeAutoProfiles = new();
    private readonly HashSet<AxisAddress> _activeAutoStartedAxes = new();

    // LIDAR effect: chỉ một transition Zone được phép chạy tại một thời điểm.
    // Khi một Zone được chấp nhận, tâm sóng được khóa trong 60 giây.
    private const double LidarPhaseSpeedMultiplier = 2.0;
    private static readonly TimeSpan LidarWaveDuration = TimeSpan.FromSeconds(60);
    private readonly SemaphoreSlim _lidarTransitionLock = new(1, 1);
    private CancellationTokenSource? _lidarTransitionCts;
    private readonly Dictionary<int, int?> _activeLidarZones = new();

    private bool _disposed;

    private sealed record AutoAxisProfile(
        AxisAddress Address,
        int ClusterId,
        int LayerIndex,
        int LocalColumn,
        double PhaseOffsetRevolutions,
        int PhaseOffsetPulses,
        ushort SpeedRpm,
        ushort AccelerationTime,
        ushort DecelerationTime,
        int PulsesPerRevolution);

    private sealed record LineSerialSettings(
        string PortName,
        int BaudRate);

    public Em2RsModbusService(ApplicationState state)
    {
        _state = state ?? throw new ArgumentNullException(nameof(state));

        _modeSettingsFilePath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "WaveMotionControl",
            "driver-mode-settings.json");

        LoadModeSettingsFromDisk();

        for (var line = 1; line <= 4; line++)
        {
            _lineLocks[line] = new SemaphoreSlim(1, 1);
        }
    }

    #region Kết nối và cấu hình DI5

    public async Task ConnectLineAsync(
        int line,
        string portName,
        int baudRate,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ValidateLine(line);

        if (string.IsNullOrWhiteSpace(portName))
        {
            throw new ArgumentException("Tên cổng COM không được để trống.", nameof(portName));
        }

        // Lưu cấu hình trước khi mở cổng để watchdog có thể tự kết nối lại
        // nếu USB-RS485 bị reset tạm thời.
        _lineSerialSettings[line] = new LineSerialSettings(
            portName.Trim(),
            baudRate);

        await StopLinePollingAsync(line).ConfigureAwait(false);
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);

        SerialPort? openedPort = null;

        try
        {
            ClosePortWithoutLock(line);

            _state.WriteLog(
                LogLevel.Info,
                $"[RS485] Line {line}: mở {portName}, {baudRate} bps, quét Slave 1..16.");

            openedPort = CreateSerialPort(portName.Trim(), baudRate);
            openedPort.Open();
            openedPort.DiscardInBuffer();
            openedPort.DiscardOutBuffer();
            _ports[line] = openedPort;

            var connection = _state.Lines[line - 1];
            connection.PortName = portName;
            connection.BaudRate = baudRate;
            connection.IsConnected = false;

            var verifiedCount = 0;
            var axesOnLine = _state.GetAxesForLine(line).ToArray();

            foreach (var axis in axesOnLine)
            {
                cancellationToken.ThrowIfCancellationRequested();

                var slaveId = checked((byte)axis.Address.SlaveId);
                var responded = await ProbeSlaveModbusAsync(
                    openedPort,
                    slaveId,
                    cancellationToken).ConfigureAwait(false);

                if (!responded)
                {
                    axis.State = AxisMotionState.Offline;
                    axis.VelocityRpm = 0;
                    axis.LastCommand = "NO_MODBUS_RESPONSE";
                    axis.AlarmText = string.Empty;
                    continue;
                }

                verifiedCount++;

                // Chỉ cấu hình mapping DI5. Không Force Enable toàn bộ driver khi Connect,
                // vì các driver chưa gắn motor sẽ báo lỗi khóa trục/quá dòng.
                var di5Ready = await EnsureDi5HomeInputOnConnectAsync(
                    openedPort,
                    axis.Address,
                    cancellationToken).ConfigureAwait(false);

                var alarm = await ReadSingleRegisterOnOpenPortAsync(
                    openedPort,
                    slaveId,
                    AlarmRegister,
                    cancellationToken).ConfigureAwait(false);

                if (alarm == 0)
                {
                    axis.State = AxisMotionState.Online;
                    axis.AlarmText = string.Empty;
                    axis.LastCommand = di5Ready
                        ? "CONNECTED_DI5_HOME_READY"
                        : "DI5_CONFIGURED_RESTART_REQUIRED";
                }
                else
                {
                    axis.State = AxisMotionState.Alarm;
                    axis.AlarmText = DescribeAlarm(alarm);
                    axis.LastCommand = $"CONNECTED_ALARM_0x{alarm:X4}";
                }
            }

            if (verifiedCount == 0)
            {
                connection.IsConnected = false;

                var message =
                    $"[RS485] Line {line}: không có driver nào phản hồi trên {portName}.";
                _state.WriteLog(LogLevel.Error, message);
                throw new InvalidOperationException(message);
            }

            connection.IsConnected = true;
            _lineReconnectFailures[line] = 0;
            _state.NotifyStateChanged();
            _state.WriteLog(
                LogLevel.Ok,
                $"[RS485] Line {line}: kết nối thành công {verifiedCount}/16 driver. " +
                "Connect không tự Enable motor.");
        }
        catch (Exception ex)
        {
            var connection = _state.Lines[line - 1];
            connection.IsConnected = false;

            if (_ports.TryGetValue(line, out var registeredPort) &&
                ReferenceEquals(registeredPort, openedPort))
            {
                _ports.TryRemove(line, out _);
                try
                {
                    if (registeredPort.IsOpen)
                    {
                        registeredPort.Close();
                    }
                }
                catch
                {
                    // Tiếp tục giải phóng tài nguyên.
                }
                registeredPort.Dispose();
            }
            else if (openedPort is not null)
            {
                try
                {
                    if (openedPort.IsOpen)
                    {
                        openedPort.Close();
                    }
                }
                catch
                {
                    // Tiếp tục giải phóng tài nguyên.
                }
                openedPort.Dispose();
            }

            _state.WriteLog(
                LogLevel.Error,
                $"[RS485] Line {line}: kết nối thất bại — {ex.Message}");
            throw;
        }
        finally
        {
            _lineLocks[line].Release();
        }

        StartLinePolling(line);
    }

    private static async Task<bool> ProbeSlaveModbusAsync(
        SerialPort port,
        byte slaveId,
        CancellationToken cancellationToken)
    {
        try
        {
            _ = await ReadSingleRegisterOnOpenPortAsync(
                port,
                slaveId,
                MotionStatusRegister,
                cancellationToken).ConfigureAwait(false);
            return true;
        }
        catch
        {
            return false;
        }
    }

    private async Task<bool> EnsureDi5HomeInputOnConnectAsync(
        SerialPort port,
        AxisAddress address,
        CancellationToken cancellationToken)
    {
        var slaveId = checked((byte)address.SlaveId);
        var mappingChanged = false;

        try
        {
            // Loại bỏ ORG bị gán trùng ở DI khác. Manual báo lỗi "Repeated settings
            // of input function" nếu cùng chức năng được gán nhiều lần.
            for (var pin = 1; pin <= 7; pin++)
            {
                var register = DigitalInputFunctionRegisters[pin];
                var current = await ReadSingleRegisterOnOpenPortAsync(
                    port,
                    slaveId,
                    register,
                    cancellationToken).ConfigureAwait(false);

                if (pin != 5 && IsOrgFunction(current))
                {
                    await WriteSingleRegisterOnOpenPortAsync(
                        port,
                        slaveId,
                        register,
                        DiFunctionInvalid,
                        cancellationToken).ConfigureAwait(false);
                    mappingChanged = true;
                }
            }

            var di5Current = await ReadSingleRegisterOnOpenPortAsync(
                port,
                slaveId,
                DigitalInputFunctionRegisters[5],
                cancellationToken).ConfigureAwait(false);

            if (di5Current != DiFunctionOrgNo)
            {
                await WriteSingleRegisterOnOpenPortAsync(
                    port,
                    slaveId,
                    DigitalInputFunctionRegisters[5],
                    DiFunctionOrgNo,
                    cancellationToken).ConfigureAwait(false);
                mappingChanged = true;
            }

            if (!mappingChanged)
            {
                return true;
            }

            await SaveAndVerifyOnOpenPortAsync(
                port,
                slaveId,
                CommandSaveMappings,
                cancellationToken).ConfigureAwait(false);

            _state.WriteLog(
                LogLevel.Warning,
                $"[DI5 CONFIG] Driver {address.DisplayId}: đã đặt DI5=ORG N.O. " +
                "và xóa ORG bị gán trùng. Cần tắt/bật nguồn driver một lần để mapping có hiệu lực.");

            return false;
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Error,
                $"[DI5 CONFIG] Driver {address.DisplayId}: {ex.Message}");
            return false;
        }
    }

    public async Task DisconnectLineAsync(
        int line,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ValidateLine(line);

        await StopLinePollingAsync(line).ConfigureAwait(false);
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);

        try
        {
            ClosePortWithoutLock(line);
            _lineSerialSettings.TryRemove(line, out _);
            _lastReconnectAttemptUtc.TryRemove(line, out _);
            _lineReconnectFailures.TryRemove(line, out _);

            var connection = _state.Lines[line - 1];
            connection.IsConnected = false;

            foreach (var axis in _state.GetAxesForLine(line))
            {
                await CancelJogAsync(axis.Address).ConfigureAwait(false);
                axis.State = AxisMotionState.Offline;
                axis.VelocityRpm = 0;
                axis.LastCommand = "DISCONNECTED";
                axis.AlarmText = string.Empty;
            }

            _state.NotifyStateChanged();
            _state.WriteLog(LogLevel.Warning, $"[RS485] Line {line}: đã ngắt kết nối.");
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    #endregion

    #region Home DI5

    public async Task HomeAsync(
        IEnumerable<AxisAddress> axes,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(axes);

        var targets = axes
            .Distinct()
            .Where(address => _state.GetAxis(address).IsOnline)
            .ToArray();

        if (targets.Length == 0)
        {
            _state.WriteLog(LogLevel.Warning, "[DI5 HOME] Không có driver Online để Home.");
            return;
        }

        var affectedLines = targets
            .Select(address => address.Line)
            .Distinct()
            .ToArray();

        foreach (var line in affectedLines)
        {
            await StopLinePollingAsync(line).ConfigureAwait(false);
        }

        try
        {
            var started = new List<AxisAddress>();

            foreach (var address in targets)
            {
                cancellationToken.ThrowIfCancellationRequested();
                await CancelJogAsync(address).ConfigureAwait(false);

                var axis = _state.GetAxis(address);
                var slaveId = checked((byte)address.SlaveId);

                try
                {
                    var alarm = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        AlarmRegister,
                        cancellationToken).ConfigureAwait(false);

                    if (alarm != 0)
                    {
                        alarm = await TryResetAlarmAndReadBackAsync(
                            address,
                            alarm,
                            cancellationToken).ConfigureAwait(false);
                    }

                    if (alarm != 0)
                    {
                        throw new InvalidOperationException(
                            $"Alarm 0x{alarm:X4}: {DescribeAlarm(alarm)}");
                    }

                    var di5Function = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        DigitalInputFunctionRegisters[5],
                        cancellationToken).ConfigureAwait(false);

                    if (!IsOrgFunction(di5Function))
                    {
                        throw new InvalidOperationException(
                            $"DI5 chưa phải ORG. Giá trị hiện tại 0x{di5Function:X4}. " +
                            "Hãy Connect lại và tắt/bật nguồn driver sau khi lưu mapping.");
                    }

                    // Áp dụng đúng dòng HOME và toàn bộ profile chuyển động HOME
                    // đã cài ở màn hình Setting trước khi Enable.
                    var appliedHomeCurrent = await ApplyConfiguredCurrentAsync(
                        address,
                        DriverOperatingMode.Home,
                        cancellationToken).ConfigureAwait(false);
                    var homeMotion = await ApplyConfiguredHomeMotionAsync(
                        address,
                        cancellationToken).ConfigureAwait(false);

                    // Chỉ Force Enable đúng driver đang được yêu cầu Home.
                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        ForcedEnableRegister,
                        0x0001,
                        cancellationToken).ConfigureAwait(false);

                    // Giữ nguyên chiều Home đã cài trong bit0, luôn chọn Home Switch
                    // và dừng tại vị trí cảm biến (bit2=1, bit1=0).
                    var currentMode = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeModeRegister,
                        cancellationToken).ConfigureAwait(false);
                    var homeMode = (ushort)((currentMode & 0x0001) | 0x0004);

                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeModeRegister,
                        homeMode,
                        cancellationToken).ConfigureAwait(false);

                    var homeFast = homeMotion.FastSpeedRpm;
                    var homeSlow = homeMotion.SlowSpeedRpm;
                    var homeAcc = homeMotion.AccelerationMsPer1000Rpm;
                    var homeDec = homeMotion.DecelerationMsPer1000Rpm;

                    var diBefore = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        DigitalInputStatusRegister,
                        cancellationToken).ConfigureAwait(false);

                    axis.State = AxisMotionState.Homing;
                    axis.VelocityRpm = homeFast;
                    axis.LastCommand = "HOME_DI5_RUNNING";
                    axis.AlarmText = string.Empty;
                    _state.NotifyStateChanged();

                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        PrControlRegister,
                        CommandHome,
                        cancellationToken).ConfigureAwait(false);

                    started.Add(address);
                    _state.WriteLog(
                        LogLevel.Info,
                        $"[DI5 HOME] {address.DisplayId}: START, DI=0x{diBefore:X4}, " +
                        $"Current={appliedHomeCurrent:0.0}A, Fast={homeFast} rpm, " +
                        $"Slow={homeSlow} rpm, Acc={homeAcc}, Dec={homeDec}.");
                }
                catch (OperationCanceledException)
                {
                    axis.State = AxisMotionState.Online;
                    axis.VelocityRpm = 0;
                    axis.LastCommand = "HOME_CANCELLED";
                    _state.NotifyStateChanged();
                    throw;
                }
                catch (Exception ex)
                {
                    axis.State = AxisMotionState.Alarm;
                    axis.VelocityRpm = 0;
                    axis.LastCommand = "HOME_START_ERROR";
                    axis.AlarmText = ex.Message;
                    _state.NotifyStateChanged();
                    _state.WriteLog(
                        LogLevel.Error,
                        $"[DI5 HOME] Driver {address.DisplayId}: không khởi động được Home — {ex.Message}");
                }
            }

            await Task.WhenAll(started.Select(address =>
                WaitForDi5HomeCompleteAsync(
                    address,
                    TimeSpan.FromSeconds(60),
                    cancellationToken))).ConfigureAwait(false);
        }
        finally
        {
            foreach (var line in affectedLines)
            {
                if (_lineSerialSettings.ContainsKey(line) || IsLinePortOpen(line))
                {
                    StartLinePolling(line);
                }
            }
        }
    }

    private async Task WaitForDi5HomeCompleteAsync(
        AxisAddress address,
        TimeSpan timeout,
        CancellationToken cancellationToken)
    {
        var axis = _state.GetAxis(address);
        var slaveId = checked((byte)address.SlaveId);
        var deadline = DateTime.UtcNow + timeout;
        ushort previousStatus = ushort.MaxValue;
        ushort previousDi = ushort.MaxValue;
        ushort lastAlarm = 0;
        ushort lastDi = 0;
        var loopIndex = 0;
        var consecutiveCommunicationFailures = 0;

        try
        {
            while (DateTime.UtcNow < deadline)
            {
                cancellationToken.ThrowIfCancellationRequested();
                loopIndex++;

                ushort status;
                try
                {
                    // Status là thanh ghi quan trọng nhất. Alarm và DI chỉ đọc định kỳ
                    // để giảm tải bus RS485 trong lúc motor đang Home.
                    status = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        MotionStatusRegister,
                        cancellationToken).ConfigureAwait(false);

                    if ((status & 0x0001) != 0 || loopIndex % 4 == 1)
                    {
                        lastAlarm = await ReadRegisterCheckedAsync(
                            address.Line,
                            slaveId,
                            AlarmRegister,
                            cancellationToken).ConfigureAwait(false);
                    }

                    if (loopIndex % 4 == 1)
                    {
                        lastDi = await ReadRegisterCheckedAsync(
                            address.Line,
                            slaveId,
                            DigitalInputStatusRegister,
                            cancellationToken).ConfigureAwait(false);
                    }

                    if (consecutiveCommunicationFailures > 0)
                    {
                        _state.WriteLog(
                            LogLevel.Ok,
                            $"[HOME MONITOR] {address.DisplayId}: truyền thông đã phục hồi.");
                    }

                    consecutiveCommunicationFailures = 0;
                }
                catch (Exception ex) when (IsTransientModbusException(ex))
                {
                    consecutiveCommunicationFailures++;

                    if (consecutiveCommunicationFailures == 1 ||
                        consecutiveCommunicationFailures ==
                        HomeMonitorMaxConsecutiveCommunicationFailures)
                    {
                        _state.WriteLog(
                            LogLevel.Warning,
                            $"[HOME MONITOR] {address.DisplayId}: mất phản hồi tạm thời " +
                            $"{consecutiveCommunicationFailures}/" +
                            $"{HomeMonitorMaxConsecutiveCommunicationFailures} — {ex.Message}");
                    }

                    if (consecutiveCommunicationFailures >=
                        HomeMonitorMaxConsecutiveCommunicationFailures)
                    {
                        throw new IOException(
                            $"Mất truyền thông liên tiếp khi Home " +
                            $"({consecutiveCommunicationFailures} lần). {ex.Message}",
                            ex);
                    }

                    await Task.Delay(
                        HomeMonitorPollIntervalMs,
                        cancellationToken).ConfigureAwait(false);
                    continue;
                }

                if (status != previousStatus || lastDi != previousDi)
                {
                    _state.WriteLog(
                        LogLevel.Info,
                        $"[HOME MONITOR] {address.DisplayId}: Status=0x{status:X4}, " +
                        $"Alarm=0x{lastAlarm:X4}, DI=0x{lastDi:X4}, " +
                        $"Running={(status >> 2) & 1}, HomeDone={(status >> 6) & 1}.");
                    previousStatus = status;
                    previousDi = lastDi;
                }

                if (lastAlarm != 0 || (status & 0x0001) != 0)
                {
                    throw new InvalidOperationException(
                        $"Alarm 0x{lastAlarm:X4}: {DescribeAlarm(lastAlarm)}");
                }

                if ((status & 0x0040) != 0)
                {
                    axis.State = AxisMotionState.Homed;
                    axis.VelocityRpm = 0;
                    axis.PositionRevolutions = 0;
                    axis.LastCommand = "HOME_OK_DI5";
                    axis.AlarmText = string.Empty;
                    _state.NotifyStateChanged();
                    _state.WriteLog(
                        LogLevel.Ok,
                        $"[DI5 HOME] Driver {address.DisplayId}: HOME DONE, " +
                        "motor đã dừng và vị trí = 0.");
                    return;
                }

                await Task.Delay(
                    HomeMonitorPollIntervalMs,
                    cancellationToken).ConfigureAwait(false);
            }

            await TryQuickStopHomeAsync(address).ConfigureAwait(false);
            axis.State = AxisMotionState.Alarm;
            axis.VelocityRpm = 0;
            axis.LastCommand = "HOME_TIMEOUT";
            axis.AlarmText = "Quá thời gian tìm DI5";
            _state.NotifyStateChanged();
            _state.WriteLog(
                LogLevel.Error,
                $"[DI5 HOME] Driver {address.DisplayId}: quá 60 giây chưa thấy DI5, " +
                "đã Quick Stop.");
        }
        catch (OperationCanceledException)
        {
            await TryQuickStopHomeAsync(address).ConfigureAwait(false);
            axis.State = AxisMotionState.Online;
            axis.VelocityRpm = 0;
            axis.LastCommand = "HOME_CANCELLED";
            _state.NotifyStateChanged();
            throw;
        }
        catch (Exception ex)
        {
            await TryQuickStopHomeAsync(address).ConfigureAwait(false);
            axis.State = AxisMotionState.Alarm;
            axis.VelocityRpm = 0;
            axis.LastCommand = "HOME_ERROR";
            axis.AlarmText = ex.Message;
            _state.NotifyStateChanged();
            _state.WriteLog(
                LogLevel.Error,
                $"[DI5 HOME] Driver {address.DisplayId}: {ex.Message}");
        }
    }

    private async Task TryQuickStopHomeAsync(AxisAddress address)
    {
        try
        {
            await WriteRegisterCheckedAsync(
                address.Line,
                checked((byte)address.SlaveId),
                PrControlRegister,
                CommandQuickStop,
                CancellationToken.None).ConfigureAwait(false);
        }
        catch
        {
            // Không che mất lỗi Home ban đầu.
        }
    }

    #endregion

    #region JOG và chạy tương đối

    public async Task StartJogAsync(
        AxisAddress axisAddress,
        JogDirection direction,
        int speedRpm,
        int acceleration,
        int deceleration,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        var axis = RequireOnline(axisAddress);
        await CancelJogAsync(axisAddress).ConfigureAwait(false);
        await EnsureNoAlarmAsync(axisAddress, cancellationToken).ConfigureAwait(false);

        var slaveId = checked((byte)axisAddress.SlaveId);
        var safeSpeed = (ushort)Math.Clamp(speedRpm, 1, 5000);
        var safeAcceleration = (ushort)Math.Clamp(acceleration, 1, 10_000);
        var safeDeceleration = (ushort)Math.Clamp(deceleration, 1, 10_000);

        var appliedManualCurrent = await ApplyConfiguredCurrentAsync(
            axisAddress,
            DriverOperatingMode.Manual,
            cancellationToken).ConfigureAwait(false);

        await WriteRegisterCheckedAsync(
            axisAddress.Line,
            slaveId,
            ForcedEnableRegister,
            0x0001,
            cancellationToken).ConfigureAwait(false);
        await WriteRegisterCheckedAsync(
            axisAddress.Line,
            slaveId,
            JogSpeedRegister,
            safeSpeed,
            cancellationToken).ConfigureAwait(false);
        await WriteRegisterCheckedAsync(
            axisAddress.Line,
            slaveId,
            JogAccelerationRegister,
            safeAcceleration,
            cancellationToken).ConfigureAwait(false);
        await WriteRegisterCheckedAsync(
            axisAddress.Line,
            slaveId,
            JogDecelerationRegister,
            safeDeceleration,
            cancellationToken).ConfigureAwait(false);

        var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        Task task;

        lock (_jogSync)
        {
            _jogCts[axisAddress] = cts;
            task = Task.Run(() => JogLoopAsync(axisAddress, direction, cts.Token));
            _jogTasks[axisAddress] = task;
        }

        axis.State = direction == JogDirection.Forward
            ? AxisMotionState.JoggingForward
            : AxisMotionState.JoggingReverse;
        axis.VelocityRpm = direction == JogDirection.Forward ? safeSpeed : -safeSpeed;
        axis.LastCommand = direction == JogDirection.Forward ? "JOG_CW" : "JOG_CCW";
        axis.AlarmText = string.Empty;
        _state.NotifyStateChanged();
        _state.WriteLog(
            LogLevel.Info,
            $"[JOG] {axisAddress.DisplayId}: {direction}, {safeSpeed} rpm, " +
            $"Current={appliedManualCurrent:0.0}A, Acc={safeAcceleration}, Dec={safeDeceleration}.");
    }

    private async Task JogLoopAsync(
        AxisAddress address,
        JogDirection direction,
        CancellationToken cancellationToken)
    {
        var command = direction == JogDirection.Forward
            ? (ushort)0x4001
            : (ushort)0x4002;

        try
        {
            // Manual yêu cầu chu kỳ kích JOG qua RS485 nhỏ hơn 50 ms
            // để chuyển động liên tục.
            while (!cancellationToken.IsCancellationRequested)
            {
                cancellationToken.ThrowIfCancellationRequested();
                await WriteRegisterCheckedAsync(
                    address.Line,
                    checked((byte)address.SlaveId),
                    SaveControlRegister,
                    command,
                    cancellationToken).ConfigureAwait(false);
                await Task.Delay(30, cancellationToken).ConfigureAwait(false);
            }
        }
        catch (OperationCanceledException)
        {
            // Dừng bình thường.
        }
        catch (Exception ex)
        {
            var axis = _state.GetAxis(address);
            axis.State = AxisMotionState.Alarm;
            axis.VelocityRpm = 0;
            axis.LastCommand = "JOG_COMM_ERROR";
            axis.AlarmText = ex.Message;
            _state.NotifyStateChanged();
            _state.WriteLog(LogLevel.Error, $"[JOG] {address.DisplayId}: {ex.Message}");
        }
    }

    public async Task StopAxisAsync(
        AxisAddress axisAddress,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        var axis = _state.GetAxis(axisAddress);
        if (!axis.IsOnline)
        {
            return;
        }

        await CancelJogAsync(axisAddress).ConfigureAwait(false);
        await WriteRegisterCheckedAsync(
            axisAddress.Line,
            checked((byte)axisAddress.SlaveId),
            PrControlRegister,
            CommandQuickStop,
            cancellationToken).ConfigureAwait(false);

        axis.VelocityRpm = 0;
        axis.State = axis.State == AxisMotionState.Homed
            ? AxisMotionState.Homed
            : AxisMotionState.Online;
        axis.LastCommand = "QUICK_STOP";
        _state.NotifyStateChanged();
    }

    public async Task MoveRelativeRevolutionsAsync(
        AxisAddress axisAddress,
        double signedRevolutions,
        int speedRpm,
        int pulsesPerRevolution,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        var axis = RequireOnline(axisAddress);
        await CancelJogAsync(axisAddress).ConfigureAwait(false);
        await EnsureNoAlarmAsync(axisAddress, cancellationToken).ConfigureAwait(false);

        // Pulse/vòng truyền từ trang Manual được giữ trong chữ ký để tương thích
        // IRs485Service cũ, nhưng giá trị thực dùng cho mọi chức năng được lấy
        // duy nhất từ phần SETTING.
        var configuredPulsesPerRevolution = GetAxisPulsesPerRevolution(axisAddress);
        if (pulsesPerRevolution > 0 &&
            pulsesPerRevolution != configuredPulsesPerRevolution)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[MOVE] {axisAddress.DisplayId}: bỏ qua Pulse/vòng={pulsesPerRevolution:N0} " +
                $"trên trang Manual; sử dụng SETTING={configuredPulsesPerRevolution:N0}.");
        }

        var targetPulsesLong = (long)Math.Round(
            signedRevolutions * configuredPulsesPerRevolution);
        targetPulsesLong = Math.Clamp(targetPulsesLong, int.MinValue, int.MaxValue);
        var targetPulses = (int)targetPulsesLong;
        var slaveId = checked((byte)axisAddress.SlaveId);

        var appliedManualCurrent = await ApplyConfiguredCurrentAsync(
            axisAddress,
            DriverOperatingMode.Manual,
            cancellationToken).ConfigureAwait(false);

        await WriteRegisterCheckedAsync(
            axisAddress.Line,
            slaveId,
            ForcedEnableRegister,
            0x0001,
            cancellationToken).ConfigureAwait(false);

        var acceleration = await ReadRegisterCheckedAsync(
            axisAddress.Line,
            slaveId,
            Pr0AccelerationRegister,
            cancellationToken).ConfigureAwait(false);
        var deceleration = await ReadRegisterCheckedAsync(
            axisAddress.Line,
            slaveId,
            Pr0DecelerationRegister,
            cancellationToken).ConfigureAwait(false);

        if (acceleration == 0) acceleration = 100;
        if (deceleration == 0) deceleration = 100;

        var values = BuildPr0Command(
            Pr0RelativeMode,
            targetPulses,
            (ushort)Math.Clamp(speedRpm, 1, 5000),
            acceleration,
            deceleration);

        await WriteMultipleRegistersCheckedAsync(
            axisAddress.Line,
            slaveId,
            Pr0ModeRegister,
            values,
            cancellationToken).ConfigureAwait(false);

        axis.State = AxisMotionState.Moving;
        axis.VelocityRpm = signedRevolutions >= 0
            ? Math.Abs(speedRpm)
            : -Math.Abs(speedRpm);
        axis.LastCommand = $"MOVE_REL_{targetPulses}_P";
        _state.NotifyStateChanged();
        _state.WriteLog(
            LogLevel.Info,
            $"[MOVE] {axisAddress.DisplayId}: {signedRevolutions:0.###} vòng, " +
            $"{targetPulses:N0} pulse, PPR={configuredPulsesPerRevolution:N0}, " +
            $"Current={appliedManualCurrent:0.0}A, {speedRpm} rpm.");
    }

    #endregion


    #region MANUAL - test con trượt gần đều bằng 16 PR tự Jump

    public UniformSliderMotionPlan PreviewUniformSliderMotion(
        AxisAddress axisAddress,
        UniformSliderMotionSettings settings)
    {
        ThrowIfDisposed();

        return UniformSliderMotionPlanner.Build(
            settings,
            GetAxisPulsesPerRevolution(axisAddress));
    }

    public async Task SetUniformMechanicalZeroAsync(
        AxisAddress axisAddress,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        var axis = RequireOnline(axisAddress);

        lock (_autoSync)
        {
            if (_autoCts is not null)
            {
                throw new InvalidOperationException(
                    "AUTO đang chạy. Hãy dừng AUTO trước khi đặt gốc cơ khí.");
            }
        }

        await CancelJogAsync(axisAddress).ConfigureAwait(false);

        var line = axisAddress.Line;
        var slaveId = checked((byte)axisAddress.SlaveId);
        await StopLinePollingAsync(line).ConfigureAwait(false);

        try
        {
            await EnsureNoAlarmAsync(
                axisAddress,
                cancellationToken).ConfigureAwait(false);

            // Motor phải đứng yên trước khi đặt tọa độ hiện tại thành 0.
            await WriteRegisterCheckedAsync(
                line,
                slaveId,
                PrControlRegister,
                CommandQuickStop,
                cancellationToken).ConfigureAwait(false);
            await Task.Delay(120, cancellationToken).ConfigureAwait(false);

            // Theo manual: ghi 0x0021 vào 0x6002 để đặt vị trí hiện tại = 0.
            await WriteRegisterCheckedAsync(
                line,
                slaveId,
                PrControlRegister,
                CommandSetCurrentPointZero,
                cancellationToken).ConfigureAwait(false);
            await Task.Delay(100, cancellationToken).ConfigureAwait(false);

            var positionWords = await ReadRegistersCheckedAsync(
                line,
                slaveId,
                ActualPositionRegister,
                2,
                cancellationToken).ConfigureAwait(false);
            var rawPosition = CombineSigned32(
                positionWords[0],
                positionWords[1]);

            if (Math.Abs((long)rawPosition) > 2)
            {
                throw new InvalidOperationException(
                    $"Đặt gốc không thành công: vị trí đọc lại còn {rawPosition} pulse.");
            }

            axis.State = AxisMotionState.Homed;
            axis.PositionRevolutions = 0;
            axis.VelocityRpm = 0;
            axis.LastCommand = "MECHANICAL_ZERO_SET";
            axis.AlarmText = string.Empty;
            _state.NotifyStateChanged();
            _state.WriteLog(
                LogLevel.Ok,
                $"[UNIFORM ZERO] {axisAddress.DisplayId}: vị trí hiện tại đã được đặt = 0 pulse.");
        }
        finally
        {
            if (_lineSerialSettings.ContainsKey(line) || IsLinePortOpen(line))
            {
                StartLinePolling(line);
            }
        }
    }

    public async Task StartUniformSliderMotionAsync(
        AxisAddress axisAddress,
        UniformSliderMotionSettings settings,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(settings);

        var axis = RequireOnline(axisAddress);

        if (axis.State != AxisMotionState.Homed)
        {
            throw new InvalidOperationException(
                $"Driver {axisAddress.DisplayId} phải HOME thành công trước khi " +
                "chạy test con trượt đều. Sau Quick Stop phải HOME lại.");
        }

        lock (_autoSync)
        {
            if (_autoCts is not null)
            {
                throw new InvalidOperationException(
                    "AUTO đang chạy. Hãy dừng AUTO trước khi chạy test MANUAL 16 PR.");
            }
        }

        await CancelJogAsync(axisAddress).ConfigureAwait(false);

        var plan = PreviewUniformSliderMotion(axisAddress, settings);
        var line = axisAddress.Line;
        var slaveId = checked((byte)axisAddress.SlaveId);

        // Khi ghi 16 x 8 thanh ghi PR, tạm dừng polling để không xen khung FC03
        // vào chuỗi cấu hình.
        await StopLinePollingAsync(line).ConfigureAwait(false);

        try
        {
            await EnsureNoAlarmAsync(
                axisAddress,
                cancellationToken).ConfigureAwait(false);

            var appliedUniformCurrent = await ApplyRequestedCurrentAsync(
                axisAddress,
                settings.PeakCurrentAmps,
                "UNIFORM 16 PR",
                cancellationToken).ConfigureAwait(false);

            await WriteRegisterCheckedAsync(
                line,
                slaveId,
                ForcedEnableRegister,
                0x0001,
                cancellationToken).ConfigureAwait(false);

            // Dừng lệnh PR còn sót trước khi thay toàn bộ bảng.
            await WriteRegisterCheckedAsync(
                line,
                slaveId,
                PrControlRegister,
                CommandQuickStop,
                cancellationToken).ConfigureAwait(false);

            for (var pathIndex = 0;
                 pathIndex < UniformSliderMotionPlanner.PathCount;
                 pathIndex++)
            {
                cancellationToken.ThrowIfCancellationRequested();

                var segment = plan.Segments[pathIndex];
                var nextPath =
                    (pathIndex + 1) %
                    UniformSliderMotionPlanner.PathCount;

                // TYPE=position (bit0), RELATIVE (bit6), OVLP (bit5),
                // JUMP (bit14), bit8..13 = PR kế tiếp. OVLP=1 giúp driver
                // chuyển tốc độ sang PR kế tiếp mà không giảm về 0 ở mỗi đoạn.
                // Đây là điều kiện quan trọng để con trượt chạy gần đều. Pause=0.
                var mode = (ushort)(
                    0x4000 |
                    ((nextPath & 0x3F) << 8) |
                    PrPathOverlapBit |
                    Pr0RelativeMode);

                var position = segment.RelativePulses;
                var pathValues = new ushort[]
                {
                    mode,
                    (ushort)((position >> 16) & 0xFFFF),
                    (ushort)(position & 0xFFFF),
                    (ushort)segment.SpeedRpm,
                    (ushort)segment.AccelerationMsPer1000Rpm,
                    (ushort)segment.DecelerationMsPer1000Rpm,
                    0x0000, // Pause
                    0x0000  // PR0: không trigger trong lúc đang ghi bảng
                };

                var startRegister = checked((ushort)(
                    Pr0ModeRegister + pathIndex * 8));

                await WriteMultipleRegistersCheckedAsync(
                    line,
                    slaveId,
                    startRegister,
                    pathValues,
                    cancellationToken).ConfigureAwait(false);
            }

            // Đọc lại PR0 và PR15 để phát hiện ghi thiếu hoặc sai khung.
            var firstReadBack = await ReadRegistersCheckedAsync(
                line,
                slaveId,
                Pr0ModeRegister,
                7,
                cancellationToken).ConfigureAwait(false);
            var lastStartRegister = checked((ushort)(
                Pr0ModeRegister +
                (UniformSliderMotionPlanner.PathCount - 1) * 8));
            var lastReadBack = await ReadRegistersCheckedAsync(
                line,
                slaveId,
                lastStartRegister,
                7,
                cancellationToken).ConfigureAwait(false);

            VerifyUniformPrSegmentReadBack(
                plan.Segments[0],
                nextPath: 1,
                firstReadBack,
                pathIndex: 0);
            VerifyUniformPrSegmentReadBack(
                plan.Segments[^1],
                nextPath: 0,
                lastReadBack,
                pathIndex: UniformSliderMotionPlanner.PathCount - 1);

            // Chỉ truyền START một lần. Từ đây PR0 -> ... -> PR15 -> PR0
            // tự Jump bên trong driver, máy tính không gửi điểm vị trí liên tục.
            await WriteRegisterCheckedAsync(
                line,
                slaveId,
                PrControlRegister,
                CommandTriggerPr0,
                cancellationToken).ConfigureAwait(false);

            var firstSpeed = plan.Segments[0].SpeedRpm;
            var firstDirection =
                Math.Sign(plan.Segments[0].RelativePulses);

            axis.State = AxisMotionState.Moving;
            axis.VelocityRpm = firstDirection * firstSpeed;
            axis.LastCommand = "MANUAL_UNIFORM_PR_LOOP";
            axis.AlarmText = string.Empty;
            _state.NotifyStateChanged();

            _state.WriteLog(
                LogLevel.Ok,
                $"[UNIFORM PR] {axisAddress.DisplayId}: START 16 PR tự Jump, " +
                $"Stroke={plan.StrokeMm:0.###} mm, " +
                $"Vslider={settings.SliderSpeedMmPerSecond:0.###} mm/s, " +
                $"Cycle={plan.DesiredCycleTimeSeconds:0.###} s, " +
                $"RPM={plan.MinimumSpeedRpm}..{plan.MaximumSpeedRpm}, " +
                $"PPR={plan.PulsesPerRevolution:N0}, " +
                $"Current={appliedUniformCurrent:0.0}A, OVLP=ON. " +
                "PR chỉ ghi RAM, không ghi EEPROM.");

            LogUniformPrTable(axisAddress, plan);
        }
        catch
        {
            try
            {
                await WriteRegisterCheckedAsync(
                    line,
                    slaveId,
                    PrControlRegister,
                    CommandQuickStop,
                    CancellationToken.None).ConfigureAwait(false);
            }
            catch
            {
                // Giữ nguyên lỗi cấu hình ban đầu.
            }

            throw;
        }
        finally
        {
            if (_lineSerialSettings.ContainsKey(line) || IsLinePortOpen(line))
            {
                StartLinePolling(line);
            }
        }
    }

    private static void VerifyUniformPrSegmentReadBack(
        UniformSliderPrSegment segment,
        int nextPath,
        IReadOnlyList<ushort> readBack,
        int pathIndex)
    {
        if (readBack.Count < 7)
        {
            throw new InvalidOperationException(
                $"Đọc lại PR{pathIndex} thiếu dữ liệu.");
        }

        var expectedMode = (ushort)(
            0x4000 |
            ((nextPath & 0x3F) << 8) |
            PrPathOverlapBit |
            Pr0RelativeMode);
        var expectedHigh =
            (ushort)((segment.RelativePulses >> 16) & 0xFFFF);
        var expectedLow =
            (ushort)(segment.RelativePulses & 0xFFFF);

        if (readBack[0] != expectedMode ||
            readBack[1] != expectedHigh ||
            readBack[2] != expectedLow ||
            readBack[3] != segment.SpeedRpm ||
            readBack[4] != segment.AccelerationMsPer1000Rpm ||
            readBack[5] != segment.DecelerationMsPer1000Rpm ||
            readBack[6] != 0)
        {
            throw new InvalidOperationException(
                $"Đọc lại PR{pathIndex} không khớp. " +
                $"Mode=0x{readBack[0]:X4}/0x{expectedMode:X4}, " +
                $"Pos=0x{readBack[1]:X4}{readBack[2]:X4}/" +
                $"0x{expectedHigh:X4}{expectedLow:X4}, " +
                $"Speed={readBack[3]}/{segment.SpeedRpm}.");
        }
    }

    private void LogUniformPrTable(
        AxisAddress axisAddress,
        UniformSliderMotionPlan plan)
    {
        for (var start = 0;
             start < UniformSliderMotionPlanner.PathCount;
             start += 8)
        {
            var text = string.Join(
                ", ",
                plan.Segments
                    .Skip(start)
                    .Take(8)
                    .Select(segment =>
                        $"PR{segment.PathIndex}=" +
                        $"{segment.RelativePulses:+#;-#;0}p@" +
                        $"{segment.SpeedRpm}rpm"));

            _state.WriteLog(
                LogLevel.Info,
                $"[UNIFORM TABLE] {axisAddress.DisplayId}: {text}");
        }
    }

    #endregion

    public async Task SetCurrentPositionAsOriginAsync(
        IEnumerable<AxisAddress> axes,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        var targets = axes.Distinct().ToArray();
        if (targets.Length == 0)
        {
            return;
        }

        var offline = targets.Where(address => !_state.GetAxis(address).IsOnline).ToArray();
        if (offline.Length > 0)
        {
            throw new InvalidOperationException(
                $"Không thể lấy gốc AUTO: driver offline {string.Join(", ", offline.Select(a => a.DisplayId))}.");
        }

        foreach (var address in targets)
        {
            cancellationToken.ThrowIfCancellationRequested();
            await SetUniformMechanicalZeroAsync(address, cancellationToken).ConfigureAwait(false);
        }

        _state.WriteLog(
            LogLevel.Ok,
            $"[AUTO ORIGIN] Đã lấy vị trí hiện tại làm pha 0 cho {targets.Length} driver.");
    }

    #region AUTO hiệu ứng theo Grid 16x16 / nhiều cụm - INTERNAL 16 PR

    /// <summary>
    /// Phiên bản AUTO thử nghiệm dùng toàn bộ 16 PR nội bộ của EM2RS.
    /// Không dùng scheduler PC để delay từng layer trong lúc hiệu ứng đang chạy.
    ///
    /// Trình tự:
    /// 1) Từ pha 0, đưa từng driver tới pha lệch cố định của layer bằng Immediate Trigger PR0.
    /// 2) Chờ toàn bộ driver đứng đúng pha.
    /// 3) Nạp PR0..PR15: 16 đoạn tương đối bằng nhau, OVLP + JUMP vòng kín.
    /// 4) Trigger PR0 cho toàn bộ driver trong một lượt ngắn; từ đây driver tự chạy nội bộ.
    ///
    /// Cách đặt pha là ảnh chụp steady-state của kiểu delay cũ tại thời điểm layer cuối vừa bắt đầu:
    /// layer đầu dẫn pha, layer cuối ở pha 0. Vì vậy không cần Task.Delay giữa các layer khi chạy.
    /// </summary>
    public async Task StartAutoAsync(
        AutoProgram program,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(program);
        ValidateAutoProgram(program);

        await CancelAutoWorkerAsync().ConfigureAwait(false);

        var requestedDrivers = program.Clusters
            .SelectMany(c => c.Cells)
            .Select(c => c.DriverId!.Value)
            .Distinct()
            .ToArray();

        var offline = requestedDrivers
            .Where(address => !_state.GetAxis(address).IsOnline)
            .ToArray();
        if (offline.Length > 0)
        {
            throw new InvalidOperationException(
                $"AUTO bị khóa vì có driver offline: {string.Join(", ", offline.Select(a => a.DisplayId))}.");
        }

        var notReferenced = requestedDrivers
            .Where(address =>
            {
                var axis = _state.GetAxis(address);
                return axis.State != AxisMotionState.Homed ||
                       Math.Abs(axis.PositionRevolutions) > 0.02;
            })
            .ToArray();
        if (notReferenced.Length > 0)
        {
            throw new InvalidOperationException(
                "AUTO 16PR yêu cầu toàn bộ driver bắt đầu ở pha 0: HOME hoặc vừa dùng " +
                "'Lấy vị trí hiện tại làm gốc'. Chưa ở pha 0: " +
                string.Join(", ", notReferenced.Select(a => a.DisplayId)) + ".");
        }

        var driverClusters = program.DriverClusters();
        var clusterLayers = program.Clusters.ToDictionary(
            cluster => cluster.Id,
            cluster => cluster.BuildWaveLayers());
        var profiles = new List<AutoAxisProfile>(requestedDrivers.Length);

        foreach (var address in requestedDrivers)
        {
            cancellationToken.ThrowIfCancellationRequested();

            var cluster = driverClusters[address];
            var layers = clusterLayers[cluster.Id];
            var localColumn = cluster.GetLocalColumn(address);

            var layerIndex = localColumn;
            double phaseOffsetRevolutions;

            if (cluster.Effect == AutoEffectType.Lidar)
            {
                // Nền LIDAR: tất cả motor cùng tốc độ, chỉ khác pha ban đầu.
                phaseOffsetRevolutions = cluster.GetLidarRandomPhase(address);
            }
            else
            {
                var layer = layers.First(x => x.Drivers.Contains(address));
                var maxLayerIndex = layers.Count == 0 ? 0 : layers.Max(x => x.Index);
                layerIndex = layer.Index;

                // Kiểu delay cũ: layer 0 đi trước, layer sau bị trễ N*offset vòng.
                // Ở steady-state, chênh pha giữa hai layer liên tiếp vẫn là offset.
                var rawPhase =
                    (maxLayerIndex - layer.Index) *
                    Math.Max(0, cluster.LayerOffsetRevolutions);
                phaseOffsetRevolutions = PositiveModuloOne(rawPhase);
            }

            var speedRps = Math.Clamp(cluster.FrequencyHz, 0.01, 5.0);
            var speedRpm = checked((ushort)Math.Clamp(
                (int)Math.Round(speedRps * 60.0),
                1,
                5000));

            var slaveId = checked((byte)address.SlaveId);
            var acc = await ReadRegisterCheckedAsync(
                address.Line,
                slaveId,
                Pr0AccelerationRegister,
                cancellationToken).ConfigureAwait(false);
            var dec = await ReadRegisterCheckedAsync(
                address.Line,
                slaveId,
                Pr0DecelerationRegister,
                cancellationToken).ConfigureAwait(false);
            if (acc == 0) acc = 100;
            if (dec == 0) dec = acc;

            var pulsesPerRevolution = GetAxisPulsesPerRevolution(address);

            var phaseOffsetPulses = (int)Math.Round(
                phaseOffsetRevolutions * pulsesPerRevolution);
            phaseOffsetPulses %= pulsesPerRevolution;

            profiles.Add(new AutoAxisProfile(
                address,
                cluster.Id,
                layerIndex,
                localColumn,
                phaseOffsetRevolutions,
                phaseOffsetPulses,
                speedRpm,
                acc,
                dec,
                pulsesPerRevolution));
        }

        var pausedLines = await PausePollingForTargetsAsync(requestedDrivers)
            .ConfigureAwait(false);

        try
        {
            // Mỗi line xử lý riêng; 4 line vẫn chạy song song.
            await Task.WhenAll(profiles
                .GroupBy(profile => profile.Address.Line)
                .Select(lineProfiles => PrepareInternal16PrLineAsync(
                    lineProfiles.OrderBy(p => p.Address.SlaveId).ToArray(),
                    cancellationToken)))
                .ConfigureAwait(false);
        }
        catch
        {
            try
            {
                await BroadcastQuickStopAsync(CancellationToken.None).ConfigureAwait(false);
            }
            catch
            {
                // Giữ lỗi chuẩn bị ban đầu.
            }
            throw;
        }
        finally
        {
            ResumePollingLines(pausedLines);
        }

        var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        lock (_autoSync)
        {
            _autoPaused = false;
            _activeAutoProgram = program;
            _activeAutoProfiles = profiles;
            _activeAutoStartedAxes.Clear();
            _activeLidarZones.Clear();
            foreach (var lidarCluster in program.Clusters.Where(c => c.Effect == AutoEffectType.Lidar))
            {
                _activeLidarZones[lidarCluster.Id] = null;
            }
            _autoCts = cts;
            // Không còn scheduler layer. Task này chỉ giữ lifecycle AUTO cho tới STOP.
            _autoTask = Task.Delay(Timeout.Infinite, cts.Token);
        }

        try
        {
            // Không delay theo layer nữa. Bốn line trigger song song; mỗi line gửi
            // các target liên tiếp trong một burst ngắn. Phase hiệu ứng đã nằm ở vị trí cơ khí.
            await TriggerAutoProfilesAsync(profiles, cancellationToken).ConfigureAwait(false);

            foreach (var profile in profiles)
            {
                lock (_autoSync)
                {
                    _activeAutoStartedAxes.Add(profile.Address);
                }

                var axis = _state.GetAxis(profile.Address);
                axis.State = AxisMotionState.Moving;
                axis.VelocityRpm = profile.SpeedRpm;
                axis.LastCommand =
                    $"AUTO_16PR_C{profile.ClusterId}_L{profile.LayerIndex}_PH{profile.PhaseOffsetRevolutions:0.###}";
                axis.AlarmText = string.Empty;
            }
            _state.NotifyStateChanged();
        }
        catch
        {
            await CancelAutoWorkerAsync().ConfigureAwait(false);
            await BroadcastQuickStopAsync(CancellationToken.None).ConfigureAwait(false);
            lock (_autoSync)
            {
                _activeAutoProgram = null;
                _activeAutoProfiles.Clear();
                _activeAutoStartedAxes.Clear();
            }
            throw;
        }

        _state.WriteLog(
            LogLevel.Ok,
            $"[AUTO 16PR INTERNAL] START {profiles.Count} driver / {program.Clusters.Count} cụm. " +
            "Không delay layer bằng PC; phase được pre-position rồi PR0..PR15 tự Jump trong EM2RS.");
    }

    private async Task PrepareInternal16PrLineAsync(
        IReadOnlyList<AutoAxisProfile> profiles,
        CancellationToken cancellationToken)
    {
        if (profiles.Count == 0)
        {
            return;
        }

        // 1) Chuẩn bị current/enable/stop và gửi lệnh pre-position cho toàn bộ driver trên line.
        foreach (var profile in profiles)
        {
            cancellationToken.ThrowIfCancellationRequested();
            await EnsureNoAlarmAsync(profile.Address, cancellationToken).ConfigureAwait(false);

            var slaveId = checked((byte)profile.Address.SlaveId);
            var appliedAutoCurrent = await ApplyConfiguredCurrentAsync(
                profile.Address,
                DriverOperatingMode.Auto,
                cancellationToken).ConfigureAwait(false);

            await WriteRegisterCheckedAsync(
                profile.Address.Line,
                slaveId,
                ForcedEnableRegister,
                0x0001,
                cancellationToken).ConfigureAwait(false);

            await WriteRegisterCheckedAsync(
                profile.Address.Line,
                slaveId,
                PrControlRegister,
                CommandQuickStop,
                cancellationToken).ConfigureAwait(false);

            if (profile.PhaseOffsetPulses != 0)
            {
                var prePositionValues = BuildPr0Command(
                    Pr0AbsoluteInterruptMode,
                    profile.PhaseOffsetPulses,
                    profile.SpeedRpm,
                    profile.AccelerationTime,
                    profile.DecelerationTime);

                // Manual V1.5 Immediate Trigger: FC10 0x6200..0x6207,
                // Pr9.07=0x0010 sẽ chạy PR0 ngay sau khi nhận đủ frame.
                await WriteMultipleRegistersCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    Pr0ModeRegister,
                    prePositionValues,
                    cancellationToken).ConfigureAwait(false);
            }

            var axis = _state.GetAxis(profile.Address);
            axis.LastCommand =
                $"AUTO_PREPHASE_C{profile.ClusterId}_L{profile.LayerIndex}_{profile.PhaseOffsetRevolutions:0.###}REV";

            _state.WriteLog(
                LogLevel.Info,
                $"[AUTO PREPHASE] {profile.Address.DisplayId}: C{profile.ClusterId} L{profile.LayerIndex}, " +
                $"Phase={profile.PhaseOffsetRevolutions:0.###} vòng ({profile.PhaseOffsetPulses:N0} pulse), " +
                $"Speed={profile.SpeedRpm} rpm, Current={appliedAutoCurrent:0.0}A.");
        }

        // 2) Chờ toàn bộ driver trên line tới phase đích rồi mới ghi đè PR0 bằng bảng loop.
        await WaitForAutoPrePositionsAsync(profiles, cancellationToken).ConfigureAwait(false);

        // 3) Ghi đủ 16 path. Mỗi path = 1/16 vòng tương đối, cùng RPM.
        foreach (var profile in profiles)
        {
            cancellationToken.ThrowIfCancellationRequested();
            await WriteInternal16PrLoopAsync(profile, cancellationToken).ConfigureAwait(false);
        }
    }

    private async Task WaitForAutoPrePositionsAsync(
        IReadOnlyList<AutoAxisProfile> profiles,
        CancellationToken cancellationToken)
    {
        var pending = profiles
            .Where(profile => profile.PhaseOffsetPulses != 0)
            .ToDictionary(profile => profile.Address, profile => profile);

        if (pending.Count == 0)
        {
            return;
        }

        // Tính timeout theo move dài nhất + biên lớn cho USB/RS485.
        var longestSeconds = pending.Values.Max(profile =>
            profile.PhaseOffsetRevolutions /
            Math.Max(1.0 / 60.0, profile.SpeedRpm / 60.0));
        var timeout = TimeSpan.FromSeconds(Math.Clamp(longestSeconds * 2.0 + 8.0, 10.0, 120.0));
        var deadline = DateTime.UtcNow + timeout;

        while (pending.Count > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();

            foreach (var pair in pending.ToArray())
            {
                var profile = pair.Value;
                var slaveId = checked((byte)profile.Address.SlaveId);

                var status = await ReadRegisterCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    MotionStatusRegister,
                    cancellationToken).ConfigureAwait(false);
                var alarm = await ReadRegisterCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    AlarmRegister,
                    cancellationToken).ConfigureAwait(false);

                if (alarm != 0 || (status & 0x0001) != 0)
                {
                    throw new InvalidOperationException(
                        $"AUTO pre-phase {profile.Address.DisplayId} báo lỗi: {DescribeAlarm(alarm)}.");
                }

                var positionWords = await ReadRegistersCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    ActualPositionRegister,
                    2,
                    cancellationToken).ConfigureAwait(false);
                var actualPulses = CombineSigned32(positionWords[0], positionWords[1]);
                var running = (status & 0x0004) != 0;
                var tolerance = Math.Max(5, profile.PulsesPerRevolution / 1000); // ~0.1% vòng

                if (!running && Math.Abs((long)actualPulses - profile.PhaseOffsetPulses) <= tolerance)
                {
                    pending.Remove(pair.Key);
                    var axis = _state.GetAxis(profile.Address);
                    axis.PositionRevolutions = actualPulses / (double)profile.PulsesPerRevolution;
                    axis.VelocityRpm = 0;
                    axis.LastCommand = "AUTO_PREPHASE_OK";
                }
            }

            if (pending.Count == 0)
            {
                break;
            }

            if (DateTime.UtcNow >= deadline)
            {
                throw new TimeoutException(
                    "AUTO 16PR timeout khi chờ đưa driver về phase ban đầu: " +
                    string.Join(", ", pending.Keys.Select(address => address.DisplayId)) + ".");
            }

            await Task.Delay(40, cancellationToken).ConfigureAwait(false);
        }
    }

    private static ushort[] BuildInternal16PrPathValues(
        AutoAxisProfile profile,
        int pathIndex)
    {
        const int segmentCount = 16;

        if (pathIndex is < 0 or >= segmentCount)
        {
            throw new ArgumentOutOfRangeException(nameof(pathIndex));
        }

        var basePulses = profile.PulsesPerRevolution / segmentCount;
        var remainder = profile.PulsesPerRevolution % segmentCount;
        var nextPath = (pathIndex + 1) % segmentCount;
        var relativePulses = basePulses + (pathIndex < remainder ? 1 : 0);

        if (relativePulses <= 0)
        {
            throw new InvalidOperationException(
                $"AUTO {profile.Address.DisplayId}: PPR quá nhỏ để chia 16 PR.");
        }

        // Manual V1.5:
        // TYPE=position, RELATIVE, OVLP, JUMP, bit8..13 = path kế tiếp.
        var mode = (ushort)(
            0x4000 |
            ((nextPath & 0x3F) << 8) |
            PrPathOverlapBit |
            Pr0RelativeMode);

        return new ushort[]
        {
            mode,
            (ushort)((relativePulses >> 16) & 0xFFFF),
            (ushort)(relativePulses & 0xFFFF),
            profile.SpeedRpm,
            profile.AccelerationTime,
            profile.DecelerationTime,
            0x0000, // Pause = 0, chuyển path liên tục
            0x0000  // Không trigger khi đang ghi bảng
        };
    }

    private static ushort[] BuildInternal16PrLoopWords(AutoAxisProfile profile)
    {
        const int segmentCount = 16;
        const int wordsPerPath = 8;
        var table = new ushort[segmentCount * wordsPerPath];

        for (var pathIndex = 0; pathIndex < segmentCount; pathIndex++)
        {
            var pathValues = BuildInternal16PrPathValues(profile, pathIndex);
            Array.Copy(
                pathValues,
                0,
                table,
                pathIndex * wordsPerPath,
                wordsPerPath);
        }

        return table;
    }

    private async Task WriteInternal16PrLoopAsync(
        AutoAxisProfile profile,
        CancellationToken cancellationToken)
    {
        const int wordsPerPath = 8;
        const int firstBulkPathCount = 15; // 15 * 8 = 120 words, nằm trong giới hạn FC10 <= 123.
        var slaveId = checked((byte)profile.Address.SlaveId);
        var table = BuildInternal16PrLoopWords(profile);

        // TỐI ƯU:
        // PR0..PR15 là 128 thanh ghi liên tiếp (0x6200..0x627F).
        // Modbus FC10 chuẩn chỉ cho tối đa 123 register / frame, nên không thể
        // gửi cả 128 word trong một frame. Thay vì 16 frame (mỗi PR một frame),
        // ta gửi:
        //   Frame 1: PR0..PR14 = 120 word
        //   Frame 2: PR15       =   8 word
        // => giảm 16 lệnh ghi xuống còn 2 lệnh ghi / driver.
        //
        // Một số firmware/adapter có thể không thích frame FC10 dài. Nếu bulk
        // write thất bại, tự động fallback về cách cũ 16 frame để giữ tương thích.
        try
        {
            var firstChunk = new ushort[firstBulkPathCount * wordsPerPath];
            var lastChunk = new ushort[wordsPerPath];
            Array.Copy(table, 0, firstChunk, 0, firstChunk.Length);
            Array.Copy(table, firstChunk.Length, lastChunk, 0, lastChunk.Length);

            await WriteMultipleRegistersCheckedAsync(
                profile.Address.Line,
                slaveId,
                Pr0ModeRegister,
                firstChunk,
                cancellationToken).ConfigureAwait(false);

            await WriteMultipleRegistersCheckedAsync(
                profile.Address.Line,
                slaveId,
                checked((ushort)(Pr0ModeRegister + firstChunk.Length)),
                lastChunk,
                cancellationToken).ConfigureAwait(false);
        }
        catch (Exception ex) when (ex is not OperationCanceledException)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[AUTO 16PR BULK] {profile.Address.DisplayId}: bulk FC10 không thành công " +
                $"({ex.Message}). Fallback 16 frame PR riêng.");

            for (var pathIndex = 0; pathIndex < 16; pathIndex++)
            {
                cancellationToken.ThrowIfCancellationRequested();

                var values = BuildInternal16PrPathValues(profile, pathIndex);
                var startRegister = checked((ushort)(Pr0ModeRegister + pathIndex * wordsPerPath));

                await WriteMultipleRegistersCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    startRegister,
                    values,
                    cancellationToken).ConfigureAwait(false);
            }
        }

        // Verify PR0 + PR15: giữ kiểm tra đầu/cuối bảng trước khi START.
        await VerifyInternal16PrLoopAsync(profile, cancellationToken).ConfigureAwait(false);

        var axis = _state.GetAxis(profile.Address);
        axis.VelocityRpm = 0;
        axis.LastCommand = $"AUTO_16PR_READY_C{profile.ClusterId}_L{profile.LayerIndex}";
        axis.AlarmText = string.Empty;

        _state.WriteLog(
            LogLevel.Info,
            $"[AUTO 16PR CONFIG FAST] {profile.Address.DisplayId}: PR0..PR15, " +
            $"bulk 120+8 word, PPR={profile.PulsesPerRevolution:N0}, " +
            $"Speed={profile.SpeedRpm} rpm, PhaseStart={profile.PhaseOffsetRevolutions:0.###} vòng.");
    }

    private async Task RestoreInternalPr0LoopEntryAsync(
        AutoAxisProfile profile,
        CancellationToken cancellationToken)
    {
        // LIDAR point-move chỉ ghi đè PR0 (0x6200..0x6207).
        // PR1..PR15 của vòng 16PR vẫn còn nguyên trong RAM driver.
        // Vì vậy sau khi re-phase chỉ cần khôi phục PR0, không nạp lại cả 16 PR.
        var slaveId = checked((byte)profile.Address.SlaveId);
        var pr0Values = BuildInternal16PrPathValues(profile, 0);

        await WriteMultipleRegistersCheckedAsync(
            profile.Address.Line,
            slaveId,
            Pr0ModeRegister,
            pr0Values,
            cancellationToken).ConfigureAwait(false);

        await VerifyInternalPrPathAsync(
            profile,
            0,
            cancellationToken).ConfigureAwait(false);

        var axis = _state.GetAxis(profile.Address);
        axis.VelocityRpm = 0;
        axis.LastCommand = $"AUTO_PR0_RESTORED_C{profile.ClusterId}_L{profile.LayerIndex}";
        axis.AlarmText = string.Empty;
    }

    private async Task RestoreInternalPr0ForProfilesAsync(
        IReadOnlyCollection<AutoAxisProfile> profiles,
        CancellationToken cancellationToken)
    {
        if (profiles.Count == 0)
        {
            return;
        }

        // Các RS485 line độc lập được xử lý song song.
        // Trong mỗi line vẫn tuần tự theo Slave ID để không tranh chấp half-duplex.
        await Task.WhenAll(
            profiles
                .GroupBy(profile => profile.Address.Line)
                .Select(async lineProfiles =>
                {
                    foreach (var profile in lineProfiles.OrderBy(p => p.Address.SlaveId))
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        await RestoreInternalPr0LoopEntryAsync(
                            profile,
                            cancellationToken).ConfigureAwait(false);
                    }
                })).ConfigureAwait(false);
    }

    private async Task VerifyInternalPrPathAsync(
        AutoAxisProfile profile,
        int pathIndex,
        CancellationToken cancellationToken)
    {
        var slaveId = checked((byte)profile.Address.SlaveId);
        var expected = BuildInternal16PrPathValues(profile, pathIndex);
        var startRegister = checked((ushort)(Pr0ModeRegister + pathIndex * 8));

        // Word thứ 8 của PR0 map tới trigger register; chỉ verify 7 word cấu hình
        // để tránh phụ thuộc trạng thái trigger đọc lại.
        var readBack = await ReadRegistersCheckedAsync(
            profile.Address.Line,
            slaveId,
            startRegister,
            7,
            cancellationToken).ConfigureAwait(false);

        if (readBack.Length < 7)
        {
            throw new InvalidOperationException(
                $"AUTO {profile.Address.DisplayId}: verify PR{pathIndex} thiếu dữ liệu.");
        }

        for (var index = 0; index < 7; index++)
        {
            if (readBack[index] != expected[index])
            {
                throw new InvalidOperationException(
                    $"AUTO {profile.Address.DisplayId}: verify PR{pathIndex} không khớp " +
                    $"word {index}: read=0x{readBack[index]:X4}, expected=0x{expected[index]:X4}.");
            }
        }
    }

    private async Task VerifyInternal16PrLoopAsync(
        AutoAxisProfile profile,
        CancellationToken cancellationToken)
    {
        await VerifyInternalPrPathAsync(
            profile,
            0,
            cancellationToken).ConfigureAwait(false);

        await VerifyInternalPrPathAsync(
            profile,
            15,
            cancellationToken).ConfigureAwait(false);
    }

    private async Task TriggerAutoProfilesAsync(
        IReadOnlyCollection<AutoAxisProfile> profiles,
        CancellationToken cancellationToken)
    {
        if (profiles.Count == 0)
        {
            return;
        }

        // Trigger burst chuyên cho AUTO: giữ lock của từng line một lần và gửi
        // FC06 trực tiếp, không chèn InterCommandDelayMs=15 ms giữa từng slave.
        // Theo manual, ở 115200 bps một message đi-về khoảng vài ms, nên cách này
        // giảm đáng kể skew START so với gọi WriteRegisterCheckedAsync từng ID.
        await Task.WhenAll(profiles
            .GroupBy(profile => profile.Address.Line)
            .Select(async lineProfiles =>
            {
                var ordered = lineProfiles.OrderBy(p => p.Address.SlaveId).ToArray();
                var line = ordered[0].Address.Line;

                await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);
                try
                {
                    var port = await GetOrReconnectPortUnderLockAsync(
                        line,
                        "AUTO 16PR START BURST",
                        cancellationToken).ConfigureAwait(false);

                    foreach (var profile in ordered)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        Exception? lastError = null;

                        for (var attempt = 1; attempt <= 2; attempt++)
                        {
                            try
                            {
                                await WriteSingleRegisterOnOpenPortAsync(
                                    port,
                                    checked((byte)profile.Address.SlaveId),
                                    PrControlRegister,
                                    CommandTriggerPr0,
                                    cancellationToken).ConfigureAwait(false);
                                lastError = null;
                                break;
                            }
                            catch (Exception ex) when (IsTransientModbusException(ex))
                            {
                                lastError = ex;
                                SafeDiscardInput(port);
                                if (attempt < 2)
                                {
                                    await Task.Delay(2, cancellationToken).ConfigureAwait(false);
                                }
                            }
                        }

                        if (lastError is not null)
                        {
                            throw new IOException(
                                $"AUTO START burst lỗi Line {line}, Slave {profile.Address.SlaveId}: " +
                                lastError.Message,
                                lastError);
                        }
                    }
                }
                finally
                {
                    _lineLocks[line].Release();
                }
            })).ConfigureAwait(false);
    }

    /// <summary>
    /// LIDAR effect (chưa nối cảm biến thật): 1 Zone = 1 cột của cụm.
    /// Khi một Zone được nhận lúc đang RANDOM:
    /// 1) Khóa Zone đó làm tâm sóng.
    /// 2) Re-phase toàn cụm một lần với tốc độ = 2 x tốc độ chạy bình thường.
    /// 3) Restore duy nhất PR0 (PR1..PR15 vẫn còn nguyên) rồi START 16PR ở tốc độ bình thường.
    /// 4) Giữ nguyên quan hệ pha và chạy sóng liên tục 60 giây, không re-phase theo Zone khác.
    /// 5) Hết 60 giây mới fade và trở về nền RANDOM.
    ///
    /// zeroBasedZoneColumn == null trong lúc 60 giây đang chạy sẽ bị bỏ qua.
    /// Các target phase là phase cơ khí tuyệt đối theo HOME/Origin, nhưng luôn chọn
    /// vị trí tương đương ở vòng hiện tại/vòng kế tiếp để giữ chiều quay Forward.
    /// </summary>
    public async Task SetLidarZoneAsync(
        int clusterId,
        int? zeroBasedZoneColumn,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();

        AutoProgram program;
        AutoCluster cluster;
        AutoAxisProfile[] profiles;
        CancellationToken autoToken;
        int? alreadyLockedZone;

        lock (_autoSync)
        {
            program = _activeAutoProgram
                ?? throw new InvalidOperationException("AUTO chưa chạy.");

            cluster = program.Clusters.FirstOrDefault(c => c.Id == clusterId)
                ?? throw new InvalidOperationException($"Không tìm thấy Cụm {clusterId} trong AUTO đang chạy.");

            if (cluster.Effect != AutoEffectType.Lidar)
            {
                throw new InvalidOperationException($"Cụm {clusterId} không dùng hiệu ứng LIDAR.");
            }

            if (_autoPaused)
            {
                throw new InvalidOperationException("AUTO đang PAUSE. Hãy RESUME trước khi kích LIDAR.");
            }

            if (_autoCts is null)
            {
                throw new InvalidOperationException("AUTO lifecycle không còn hoạt động.");
            }

            autoToken = _autoCts.Token;
            profiles = _activeAutoProfiles
                .Where(p => p.ClusterId == clusterId)
                .OrderBy(p => p.Address.Line)
                .ThenBy(p => p.Address.SlaveId)
                .ToArray();

            _activeLidarZones.TryGetValue(clusterId, out alreadyLockedZone);
        }

        if (profiles.Length == 0)
        {
            throw new InvalidOperationException($"Cụm {clusterId} không có driver LIDAR đang hoạt động.");
        }

        if (zeroBasedZoneColumn is int requestedZone &&
            (requestedZone < 0 || requestedZone >= cluster.Width))
        {
            throw new ArgumentOutOfRangeException(
                nameof(zeroBasedZoneColumn),
                $"Zone LIDAR phải từ 1 đến {cluster.Width}.");
        }

        // Trong cửa sổ 60 giây, tâm sóng đã khóa. Mọi Zone ENTER/EXIT mới đều bị bỏ qua.
        if (alreadyLockedZone is int lockedZone)
        {
            _state.WriteLog(
                LogLevel.Info,
                $"[LIDAR] Cụm {clusterId}: đang khóa tâm Zone {lockedZone + 1} trong cửa sổ 60 giây; " +
                "bỏ qua tín hiệu Zone mới/EXIT.");
            return;
        }

        // EXIT khi đang RANDOM không cần làm gì.
        if (zeroBasedZoneColumn is null)
        {
            return;
        }

        var activeZone = zeroBasedZoneColumn.Value;

        // Khóa tâm NGAY KHI chấp nhận tín hiệu đầu tiên để tín hiệu kế tiếp không đổi tâm
        // trong lúc đang re-phase.
        lock (_autoSync)
        {
            if (_activeLidarZones.TryGetValue(clusterId, out var raceZone) && raceZone is int existingZone)
            {
                _state.WriteLog(
                    LogLevel.Info,
                    $"[LIDAR] Cụm {clusterId}: Zone {existingZone + 1} đã được khóa; bỏ qua Zone {activeZone + 1}.");
                return;
            }

            _activeLidarZones[clusterId] = activeZone;
        }

        using var transitionCts = CancellationTokenSource.CreateLinkedTokenSource(
            cancellationToken,
            autoToken);

        lock (_autoSync)
        {
            _lidarTransitionCts = transitionCts;
        }

        var token = transitionCts.Token;
        await _lidarTransitionLock.WaitAsync(token).ConfigureAwait(false);
        try
        {
            token.ThrowIfCancellationRequested();

            var pausedLines = await PausePollingForTargetsAsync(
                profiles.Select(p => p.Address)).ConfigureAwait(false);

            try
            {
                await QuickStopAutoProfilesAsync(profiles, token).ConfigureAwait(false);

                var targets = profiles.ToDictionary(
                    profile => profile,
                    profile => cluster.GetLidarTargetRevolutions(activeZone, profile.LocalColumn));

                // Chỉ giai đoạn tạo lệch pha ban đầu chạy nhanh 2X.
                await MoveLidarProfilesToMechanicalPhasesAsync(
                    targets,
                    $"ZONE {activeZone + 1} / PHASE 2X",
                    token,
                    LidarPhaseSpeedMultiplier).ConfigureAwait(false);

                // Point-move chỉ ghi đè PR0. PR1..PR15 vẫn nguyên nên restore PR0 là đủ.
                await RestoreInternalPr0ForProfilesAsync(
                    profiles,
                    token).ConfigureAwait(false);

                // START lại ở tốc độ bình thường. Từ đây chỉ giữ lệch pha ban đầu và
                // tất cả motor chạy cùng tốc độ liên tục.
                await TriggerAutoProfilesAsync(profiles, token).ConfigureAwait(false);

                foreach (var profile in profiles)
                {
                    var axis = _state.GetAxis(profile.Address);
                    axis.State = AxisMotionState.Moving;
                    axis.VelocityRpm = profile.SpeedRpm;
                    axis.LastCommand = $"LIDAR_WAVE_Z{activeZone + 1}_60S_RUNNING";
                    axis.AlarmText = string.Empty;
                }

                _state.NotifyStateChanged();
                _state.WriteLog(
                    LogLevel.Ok,
                    $"[LIDAR] Cụm {clusterId}: khóa Zone {activeZone + 1}; re-phase @2X hoàn tất. " +
                    "Sóng 16PR chạy liên tục 60 giây ở tốc độ bình thường; trong thời gian này không đổi pha nữa.");
            }
            finally
            {
                ResumePollingLines(pausedLines);
            }
        }
        catch
        {
            lock (_autoSync)
            {
                if (_activeLidarZones.TryGetValue(clusterId, out var zone) && zone == activeZone)
                {
                    _activeLidarZones[clusterId] = null;
                }
            }
            throw;
        }
        finally
        {
            _lidarTransitionLock.Release();
            lock (_autoSync)
            {
                if (ReferenceEquals(_lidarTransitionCts, transitionCts))
                {
                    _lidarTransitionCts = null;
                }
            }
        }

        // Không giữ SetLidarZoneAsync treo 60 giây. Timer chạy nền và tự trả về RANDOM.
        _ = RunLidarWaveWindowAsync(clusterId, activeZone, autoToken);
    }

    private async Task RunLidarWaveWindowAsync(
        int clusterId,
        int activeZone,
        CancellationToken autoToken)
    {
        try
        {
            await Task.Delay(LidarWaveDuration, autoToken).ConfigureAwait(false);

            // Nếu AUTO đang PAUSE đúng lúc hết 60 giây thì chờ RESUME rồi mới thực hiện
            // transition trở về RANDOM, tránh tự khởi động motor trong trạng thái PAUSE.
            while (_autoPaused)
            {
                autoToken.ThrowIfCancellationRequested();
                await Task.Delay(100, autoToken).ConfigureAwait(false);
            }

            await ReturnLidarClusterToRandomAsync(
                clusterId,
                activeZone,
                autoToken).ConfigureAwait(false);
        }
        catch (OperationCanceledException) when (autoToken.IsCancellationRequested)
        {
            // AUTO STOP/QUICK STOP: lifecycle kết thúc bình thường.
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Error,
                $"[LIDAR] Cụm {clusterId}: lỗi khi kết thúc cửa sổ 60 giây: {ex.Message}");
        }
    }

    private async Task ReturnLidarClusterToRandomAsync(
        int clusterId,
        int expectedZone,
        CancellationToken cancellationToken)
    {
        AutoCluster cluster;
        AutoAxisProfile[] profiles;

        lock (_autoSync)
        {
            var program = _activeAutoProgram;
            if (program is null || _autoCts is null)
            {
                return;
            }

            if (!_activeLidarZones.TryGetValue(clusterId, out var activeZone) || activeZone != expectedZone)
            {
                return;
            }

            cluster = program.Clusters.First(c => c.Id == clusterId);
            profiles = _activeAutoProfiles
                .Where(p => p.ClusterId == clusterId)
                .OrderBy(p => p.Address.Line)
                .ThenBy(p => p.Address.SlaveId)
                .ToArray();
        }

        await _lidarTransitionLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            var pausedLines = await PausePollingForTargetsAsync(
                profiles.Select(p => p.Address)).ConfigureAwait(false);

            try
            {
                await QuickStopAutoProfilesAsync(profiles, cancellationToken).ConfigureAwait(false);

                // Fade sau khi đủ 60 giây. Trong 60 giây chạy wave không có bất kỳ re-phase nào.
                foreach (var scale in new[] { 0.66, 0.33 })
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var fadeTargets = profiles.ToDictionary(
                        profile => profile,
                        profile => cluster.GetLidarTargetRevolutions(expectedZone, profile.LocalColumn) * scale);

                    await MoveLidarProfilesToMechanicalPhasesAsync(
                        fadeTargets,
                        $"FADE {scale:0.00}",
                        cancellationToken).ConfigureAwait(false);
                }

                var randomTargets = profiles.ToDictionary(
                    profile => profile,
                    profile => PositiveModuloOne(profile.PhaseOffsetRevolutions));

                await MoveLidarProfilesToMechanicalPhasesAsync(
                    randomTargets,
                    "RETURN RANDOM PHASE",
                    cancellationToken).ConfigureAwait(false);

                await RestoreInternalPr0ForProfilesAsync(
                    profiles,
                    cancellationToken).ConfigureAwait(false);

                await TriggerAutoProfilesAsync(profiles, cancellationToken).ConfigureAwait(false);

                foreach (var profile in profiles)
                {
                    var axis = _state.GetAxis(profile.Address);
                    axis.State = AxisMotionState.Moving;
                    axis.VelocityRpm = profile.SpeedRpm;
                    axis.LastCommand = "LIDAR_RANDOM_RUNNING";
                    axis.AlarmText = string.Empty;
                }

                lock (_autoSync)
                {
                    if (_activeLidarZones.TryGetValue(clusterId, out var zone) && zone == expectedZone)
                    {
                        _activeLidarZones[clusterId] = null;
                    }
                }

                _state.NotifyStateChanged();
                _state.WriteLog(
                    LogLevel.Ok,
                    $"[LIDAR] Cụm {clusterId}: đủ 60 giây -> fade xong và trở lại RANDOM 16PR.");
            }
            finally
            {
                ResumePollingLines(pausedLines);
            }
        }
        finally
        {
            _lidarTransitionLock.Release();
        }
    }

    private async Task QuickStopAutoProfilesAsync(
        IReadOnlyCollection<AutoAxisProfile> profiles,
        CancellationToken cancellationToken)
    {
        await Task.WhenAll(profiles
            .GroupBy(profile => profile.Address.Line)
            .Select(async group =>
            {
                foreach (var profile in group.OrderBy(p => p.Address.SlaveId))
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    await WriteRegisterCheckedAsync(
                        profile.Address.Line,
                        checked((byte)profile.Address.SlaveId),
                        PrControlRegister,
                        CommandQuickStop,
                        cancellationToken).ConfigureAwait(false);
                }
            })).ConfigureAwait(false);
    }

    private async Task MoveLidarProfilesToMechanicalPhasesAsync(
        IReadOnlyDictionary<AutoAxisProfile, double> phaseTargets,
        string commandLabel,
        CancellationToken cancellationToken,
        double speedMultiplier = 1.0)
    {
        if (phaseTargets.Count == 0)
        {
            return;
        }

        var absoluteTargets = new ConcurrentDictionary<AxisAddress, int>();

        await Task.WhenAll(phaseTargets
            .GroupBy(pair => pair.Key.Address.Line)
            .Select(async lineGroup =>
            {
                foreach (var pair in lineGroup.OrderBy(x => x.Key.Address.SlaveId))
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var profile = pair.Key;
                    var phase = Math.Clamp(pair.Value, 0, 0.999999);
                    var slaveId = checked((byte)profile.Address.SlaveId);

                    await EnsureNoAlarmAsync(profile.Address, cancellationToken).ConfigureAwait(false);

                    var positionWords = await ReadRegistersCheckedAsync(
                        profile.Address.Line,
                        slaveId,
                        ActualPositionRegister,
                        2,
                        cancellationToken).ConfigureAwait(false);
                    var actualPulses = CombineSigned32(positionWords[0], positionWords[1]);

                    var phasePulses = (long)Math.Round(phase * profile.PulsesPerRevolution);
                    phasePulses = Math.Clamp(phasePulses, 0, profile.PulsesPerRevolution - 1L);

                    // Chọn cùng phase cơ khí ở vòng hiện tại / vòng kế tiếp.
                    // Nếu target đã nằm phía sau thì cộng thêm 1 vòng để giữ chiều quay Forward.
                    var cycle = (long)Math.Floor(actualPulses / (double)profile.PulsesPerRevolution);
                    var targetLong = cycle * profile.PulsesPerRevolution + phasePulses;
                    var tolerance = Math.Max(5, profile.PulsesPerRevolution / 1000);
                    if (targetLong <= (long)actualPulses + tolerance)
                    {
                        targetLong += profile.PulsesPerRevolution;
                    }

                    if (targetLong > int.MaxValue || targetLong < int.MinValue)
                    {
                        throw new InvalidOperationException(
                            $"LIDAR {profile.Address.DisplayId}: tọa độ tuyệt đối sắp vượt Int32. " +
                            "Cần rebase tọa độ trước khi tiếp tục.");
                    }

                    var targetPulses = (int)targetLong;
                    var effectiveMultiplier = double.IsFinite(speedMultiplier)
                        ? Math.Max(0.01, speedMultiplier)
                        : 1.0;
                    var moveSpeedRpm = checked((ushort)Math.Clamp(
                        (int)Math.Round(profile.SpeedRpm * effectiveMultiplier),
                        1,
                        5000));

                    var values = BuildPr0Command(
                        Pr0AbsoluteInterruptMode,
                        targetPulses,
                        moveSpeedRpm,
                        profile.AccelerationTime,
                        profile.DecelerationTime);

                    await WriteMultipleRegistersCheckedAsync(
                        profile.Address.Line,
                        slaveId,
                        Pr0ModeRegister,
                        values,
                        cancellationToken).ConfigureAwait(false);

                    absoluteTargets[profile.Address] = targetPulses;
                    var axis = _state.GetAxis(profile.Address);
                    axis.State = AxisMotionState.Moving;
                    axis.VelocityRpm = moveSpeedRpm;
                    axis.LastCommand = $"LIDAR_{commandLabel}_{phase:0.###}REV";
                }
            })).ConfigureAwait(false);

        _state.NotifyStateChanged();
        await WaitForLidarAbsoluteTargetsAsync(
            phaseTargets.Keys.ToArray(),
            absoluteTargets,
            cancellationToken).ConfigureAwait(false);

        foreach (var pair in phaseTargets)
        {
            var axis = _state.GetAxis(pair.Key.Address);
            axis.State = AxisMotionState.Moving; // AUTO vẫn active, dù đang giữ target.
            axis.VelocityRpm = 0;
            axis.LastCommand = $"LIDAR_HOLD_{commandLabel}_{pair.Value:0.###}REV";
        }
        _state.NotifyStateChanged();
    }

    private async Task WaitForLidarAbsoluteTargetsAsync(
        IReadOnlyCollection<AutoAxisProfile> profiles,
        IReadOnlyDictionary<AxisAddress, int> absoluteTargets,
        CancellationToken cancellationToken)
    {
        var pending = profiles.ToDictionary(p => p.Address, p => p);
        var deadline = DateTime.UtcNow + TimeSpan.FromSeconds(125);

        while (pending.Count > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();

            foreach (var pair in pending.ToArray())
            {
                var profile = pair.Value;
                var slaveId = checked((byte)profile.Address.SlaveId);
                var status = await ReadRegisterCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    MotionStatusRegister,
                    cancellationToken).ConfigureAwait(false);
                var alarm = await ReadRegisterCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    AlarmRegister,
                    cancellationToken).ConfigureAwait(false);

                if (alarm != 0 || (status & 0x0001) != 0)
                {
                    throw new InvalidOperationException(
                        $"LIDAR {profile.Address.DisplayId} báo lỗi: {DescribeAlarm(alarm)}.");
                }

                var positionWords = await ReadRegistersCheckedAsync(
                    profile.Address.Line,
                    slaveId,
                    ActualPositionRegister,
                    2,
                    cancellationToken).ConfigureAwait(false);
                var actualPulses = CombineSigned32(positionWords[0], positionWords[1]);
                var target = absoluteTargets[profile.Address];
                var running = (status & 0x0004) != 0;
                var tolerance = Math.Max(5, profile.PulsesPerRevolution / 1000);

                if (!running && Math.Abs((long)actualPulses - target) <= tolerance)
                {
                    pending.Remove(pair.Key);
                    var axis = _state.GetAxis(profile.Address);
                    axis.PositionRevolutions = actualPulses / (double)profile.PulsesPerRevolution;
                    axis.VelocityRpm = 0;
                }
            }

            if (pending.Count == 0)
            {
                break;
            }

            if (DateTime.UtcNow >= deadline)
            {
                throw new TimeoutException(
                    "LIDAR timeout khi chờ motor tới target: " +
                    string.Join(", ", pending.Keys.Select(a => a.DisplayId)) + ".");
            }

            await Task.Delay(40, cancellationToken).ConfigureAwait(false);
        }
    }

    public async Task PauseAutoAsync(bool paused, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();

        if (paused)
        {
            _autoPaused = true;
            await BroadcastQuickStopAsync(cancellationToken).ConfigureAwait(false);

            HashSet<AxisAddress> started;
            lock (_autoSync)
            {
                started = _activeAutoStartedAxes.ToHashSet();
            }

            foreach (var address in started)
            {
                var axis = _state.GetAxis(address);
                if (axis.State != AxisMotionState.Alarm)
                {
                    axis.State = AxisMotionState.Homed;
                }
                axis.VelocityRpm = 0;
                axis.LastCommand = "AUTO_16PR_PAUSED";
            }

            _state.NotifyStateChanged();
            _state.WriteLog(LogLevel.Info, "[AUTO 16PR] PAUSE - Quick Stop, giữ phase cơ khí hiện tại.");
            return;
        }

        List<AutoAxisProfile> resumeProfiles;
        lock (_autoSync)
        {
            resumeProfiles = _activeAutoProfiles
                .Where(p => _activeAutoStartedAxes.Contains(p.Address))
                .ToList();
        }

        await TriggerAutoProfilesAsync(resumeProfiles, cancellationToken).ConfigureAwait(false);
        foreach (var profile in resumeProfiles)
        {
            var axis = _state.GetAxis(profile.Address);
            if (axis.State != AxisMotionState.Alarm)
            {
                axis.State = AxisMotionState.Moving;
            }
            axis.VelocityRpm = profile.SpeedRpm;
            axis.LastCommand = "AUTO_16PR_RESUMED";
        }

        _autoPaused = false;
        _state.NotifyStateChanged();
        _state.WriteLog(LogLevel.Info, "[AUTO 16PR] RESUME - PR0..PR15 tiếp tục từ vị trí đang giữ.");
    }

    public async Task StopAllAsync(bool quickStop, CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        List<AutoAxisProfile> profiles;
        lock (_autoSync)
        {
            profiles = _activeAutoProfiles.ToList();
        }

        await CancelAutoWorkerAsync().ConfigureAwait(false);
        await CancelAllJogAsync().ConfigureAwait(false);
        await BroadcastQuickStopAsync(cancellationToken).ConfigureAwait(false);

        var autoAddresses = profiles.Select(profile => profile.Address).ToHashSet();
        foreach (var axis in _state.Axes.Where(axis => axis.IsOnline))
        {
            axis.VelocityRpm = 0;
            if (axis.State != AxisMotionState.Alarm && autoAddresses.Contains(axis.Address))
            {
                // STOP không HOME. Pha hiện tại được giữ; muốn START mới từ pha chuẩn
                // thì HOME hoặc bấm "Lấy vị trí hiện tại = gốc" theo quy trình AUTO.
                axis.State = AxisMotionState.Homed;
            }
            axis.LastCommand = quickStop ? "QUICK_STOP" : "AUTO_16PR_STOP";
        }

        lock (_autoSync)
        {
            _activeAutoProgram = null;
            _activeAutoProfiles.Clear();
            _activeAutoStartedAxes.Clear();
            _activeLidarZones.Clear();
        }

        _state.NotifyStateChanged();
        _state.WriteLog(
            quickStop ? LogLevel.Error : LogLevel.Ok,
            quickStop
                ? "[AUTO 16PR] QUICK STOP toàn hệ thống."
                : "[AUTO 16PR] STOP - không HOME, giữ vị trí hiện tại.");
    }

    private static double PositiveModuloOne(double value)
    {
        if (!double.IsFinite(value))
        {
            return 0;
        }

        var result = value % 1.0;
        return result < 0 ? result + 1.0 : result;
    }

    #endregion

    #region Polling trạng thái thật

    private void StartLinePolling(int line)
    {
        if (_disposed)
        {
            return;
        }

        if (_pollCts.TryGetValue(line, out var existing))
        {
            existing.Cancel();
        }

        var cts = new CancellationTokenSource();
        _pollCts[line] = cts;
        _pollTasks[line] = Task.Run(() => LinePollingLoopAsync(line, cts.Token));
    }

    private async Task StopLinePollingAsync(int line)
    {
        if (!_pollCts.Remove(line, out var cts))
        {
            return;
        }

        cts.Cancel();
        if (_pollTasks.Remove(line, out var task))
        {
            try
            {
                await task.ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                // Dừng bình thường.
            }
            catch
            {
                // Polling không được làm hỏng thao tác Connect/Disconnect/Home.
            }
        }
        cts.Dispose();
    }

    private async Task LinePollingLoopAsync(int line, CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested)
        {
            try
            {
                await Task.Delay(PollDelayMs, cancellationToken).ConfigureAwait(false);

                if (!await EnsureLineAvailableForPollingAsync(
                        line,
                        cancellationToken).ConfigureAwait(false))
                {
                    await Task.Delay(
                        PortReconnectCooldownMs,
                        cancellationToken).ConfigureAwait(false);
                    continue;
                }

                var consecutiveSlaveFailures = 0;
                foreach (var axis in _state.GetAxesForLine(line))
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var pollOk = await PollAxisStatusAsync(
                        line,
                        checked((byte)axis.Address.SlaveId),
                        axis,
                        cancellationToken).ConfigureAwait(false);

                    if (pollOk)
                    {
                        consecutiveSlaveFailures = 0;
                    }
                    else
                    {
                        consecutiveSlaveFailures++;
                        if (consecutiveSlaveFailures >=
                            PollConsecutiveSlaveFailuresBeforePortRecycle)
                        {
                            await RecycleLinePortAfterPollFailuresAsync(
                                line,
                                consecutiveSlaveFailures,
                                cancellationToken).ConfigureAwait(false);
                            break;
                        }
                    }
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                _state.WriteLog(LogLevel.Warning, $"[POLL] Line {line}: {ex.Message}");
                await Task.Delay(250, cancellationToken).ConfigureAwait(false);
            }
        }
    }

    private async Task RecycleLinePortAfterPollFailuresAsync(
        int line,
        int consecutiveFailures,
        CancellationToken cancellationToken)
    {
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            // Nhiều Slave liên tiếp cùng không phản hồi thường là USB-RS485 bị
            // treo/reset, không phải lỗi riêng một driver. Đóng object SerialPort
            // cũ để vòng polling kế tiếp mở lại cổng sạch.
            ClosePortWithoutLock(line);
            _state.Lines[line - 1].IsConnected = false;
            _state.NotifyStateChanged();
            _state.WriteLog(
                LogLevel.Warning,
                $"[POLL WATCHDOG] Line {line}: {consecutiveFailures} Slave liên tiếp " +
                "không phản hồi. Đóng cổng cũ và chuẩn bị tự kết nối lại.");
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    private async Task<bool> PollAxisStatusAsync(
        int line,
        byte slaveId,
        AxisRuntime axis,
        CancellationToken cancellationToken)
    {
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);

        try
        {
            var port = await GetOrReconnectPortUnderLockAsync(
                line,
                "POLL",
                cancellationToken,
                respectReconnectCooldown: true).ConfigureAwait(false);

            var status = await ReadSingleRegisterOnOpenPortAsync(
                port,
                slaveId,
                MotionStatusRegister,
                cancellationToken).ConfigureAwait(false);
            var alarm = await ReadSingleRegisterOnOpenPortAsync(
                port,
                slaveId,
                AlarmRegister,
                cancellationToken).ConfigureAwait(false);
            var positionWords = await ReadHoldingRegistersOnOpenPortAsync(
                port,
                slaveId,
                ActualPositionRegister,
                2,
                cancellationToken).ConfigureAwait(false);
            var velocityWords = await ReadHoldingRegistersOnOpenPortAsync(
                port,
                slaveId,
                FeedbackVelocityRegister,
                2,
                cancellationToken).ConfigureAwait(false);

            _pollFailures[axis.Address] = 0;

            var rawPosition = CombineSigned32(positionWords[0], positionWords[1]);
            var rawVelocity = CombineSigned32(velocityWords[0], velocityWords[1]);

            var pulsesPerRevolution = GetAxisPulsesPerRevolution(axis.Address);
            axis.PositionRevolutions = Math.Round(
                rawPosition / (double)pulsesPerRevolution,
                4);
            axis.VelocityRpm = (int)Math.Clamp(rawVelocity, int.MinValue, int.MaxValue);

            var fault = alarm != 0 || (status & 0x0001) != 0;
            var running = (status & 0x0004) != 0;
            var homeDone = (status & 0x0040) != 0;

            if (fault)
            {
                axis.State = AxisMotionState.Alarm;
                axis.AlarmText = DescribeAlarm(alarm);
                axis.LastCommand = $"ALARM_0x{alarm:X4}";
            }
            else
            {
                axis.AlarmText = string.Empty;

                if (axis.State == AxisMotionState.Homing && homeDone)
                {
                    axis.State = AxisMotionState.Homed;
                    axis.PositionRevolutions = 0;
                    axis.VelocityRpm = 0;
                    axis.LastCommand = "HOME_OK_DI5";
                }
                else if (running)
                {
                    if (axis.State is not AxisMotionState.Homing and
                        not AxisMotionState.JoggingForward and
                        not AxisMotionState.JoggingReverse)
                    {
                        axis.State = AxisMotionState.Moving;
                    }
                }
                else if (axis.LastCommand.StartsWith(
                             "MANUAL_UNIFORM_PR_LOOP",
                             StringComparison.Ordinal))
                {
                    // Giữa hai PR có thể có một khe rất ngắn mà bit Running=0.
                    // Giữ trạng thái Moving cho tới khi StopAxis đổi LastCommand.
                    axis.State = AxisMotionState.Moving;
                }
                else if (axis.State is AxisMotionState.Moving or
                         AxisMotionState.JoggingForward or
                         AxisMotionState.JoggingReverse)
                {
                    axis.VelocityRpm = 0;
                    axis.State = homeDone ? AxisMotionState.Homed : AxisMotionState.Online;
                }
                else if (axis.State == AxisMotionState.Offline)
                {
                    axis.State = homeDone ? AxisMotionState.Homed : AxisMotionState.Online;
                }
            }

            _state.NotifyStateChanged();
            return true;
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception ex)
        {
            var failures = _pollFailures.TryGetValue(axis.Address, out var count)
                ? count + 1
                : 1;
            _pollFailures[axis.Address] = failures;

            if (failures >= 5)
            {
                axis.State = AxisMotionState.Offline;
                axis.VelocityRpm = 0;
                axis.LastCommand = "POLL_TIMEOUT";
                axis.AlarmText = ex.Message;
                _state.NotifyStateChanged();
            }

            return false;
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    #endregion

    #region Setting, đọc lại và lưu EEPROM

    /// <summary>
    /// Lưu ba mức dòng theo chế độ và Pulse/vòng dùng chung cho HOME, MANUAL,
    /// AUTO và phần hiển thị vị trí.
    ///
    /// EM2RS chỉ có một thanh ghi dòng Peak 0x0191. Vì vậy:
    /// - Dòng HOME được ghi vào driver và lưu EEPROM làm mức mặc định khi bật nguồn.
    /// - Dòng MANUAL/AUTO được lưu trong cấu hình phần mềm và được ghi thật vào
    ///   0x0191 ngay trước khi chế độ tương ứng chạy.
    /// - Không lưu EEPROM mỗi lần chuyển chế độ để tránh ghi EEPROM liên tục.
    /// </summary>
    /// <summary>
    /// Lưu trọn bộ cấu hình Setting trong một chu kỳ truyền:
    /// DI Home, dòng giữ, dòng HOME/MANUAL/AUTO, Pulse/vòng,
    /// tốc độ/gia giảm tốc HOME và tốc độ/gia tốc AUTO.
    ///
    /// Chỉ dòng HOME và thông số chuyển động được lưu vào EEPROM driver.
    /// Dòng MANUAL/AUTO và Pulse/vòng được lưu trong profile phần mềm, rồi
    /// được áp dụng ngay trước khi chế độ tương ứng chạy.
    /// </summary>
    public async Task SaveCompleteDriverSettingsAsync(
        IEnumerable<AxisAddress> axes,
        int diPinIndex,
        bool activeLowNC,
        int standbyPercent,
        double autoSpeedRps,
        double autoAccRps2,
        DriverModeSettings settings,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(axes);
        ArgumentNullException.ThrowIfNull(settings);

        if (diPinIndex is < 1 or > 7)
        {
            throw new ArgumentOutOfRangeException(
                nameof(diPinIndex),
                "DI phải từ 1 đến 7.");
        }

        var targets = axes.Distinct().ToArray();
        if (targets.Length == 0)
        {
            return;
        }

        var normalized = NormalizeModeSettings(settings);
        var selectedDiValue = activeLowNC ? DiFunctionOrgNc : DiFunctionOrgNo;
        var standbyValue = (ushort)Math.Clamp(standbyPercent, 0, 100);
        var autoSpeedRpm = (ushort)Math.Clamp(
            (int)Math.Round(autoSpeedRps * 60.0),
            1,
            5000);
        var autoAccelerationTime =
            AccelerationRps2ToMsPer1000Rpm(autoAccRps2);

        var homeCurrentValue = CurrentAmpsToRegister(normalized.HomeCurrentAmps);
        var homeValues = new ushort[]
        {
            checked((ushort)normalized.HomeFastSpeedRpm),
            checked((ushort)normalized.HomeSlowSpeedRpm),
            checked((ushort)normalized.HomeAccelerationMsPer1000Rpm),
            checked((ushort)normalized.HomeDecelerationMsPer1000Rpm)
        };
        var autoValues = new ushort[]
        {
            autoSpeedRpm,
            autoAccelerationTime,
            autoAccelerationTime,
            0
        };

        var pausedLines = await PausePollingForTargetsAsync(targets)
            .ConfigureAwait(false);

        var success = 0;
        var failed = 0;
        var offline = 0;
        var restartRequired = 0;

        try
        {
            foreach (var address in targets)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var axis = _state.GetAxis(address);

                if (!axis.IsOnline)
                {
                    offline++;
                    continue;
                }

                try
                {
                    await CancelJogAsync(address).ConfigureAwait(false);
                    var slaveId = checked((byte)address.SlaveId);

                    var status = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        MotionStatusRegister,
                        cancellationToken).ConfigureAwait(false);

                    if ((status & 0x0004) != 0)
                    {
                        throw new InvalidOperationException(
                            "Driver đang chạy. Hãy dừng HOME/JOG/MOVE/AUTO trước khi lưu.");
                    }

                    var mappingChanged = false;

                    for (var pin = 1; pin <= 7; pin++)
                    {
                        var register = DigitalInputFunctionRegisters[pin];
                        var existing = await ReadRegisterCheckedAsync(
                            address.Line,
                            slaveId,
                            register,
                            cancellationToken).ConfigureAwait(false);

                        if (pin == diPinIndex)
                        {
                            if (existing != selectedDiValue)
                            {
                                await WriteRegisterCheckedAsync(
                                    address.Line,
                                    slaveId,
                                    register,
                                    selectedDiValue,
                                    cancellationToken).ConfigureAwait(false);
                                mappingChanged = true;
                            }
                        }
                        else if (IsOrgFunction(existing))
                        {
                            await WriteRegisterCheckedAsync(
                                address.Line,
                                slaveId,
                                register,
                                DiFunctionInvalid,
                                cancellationToken).ConfigureAwait(false);
                            mappingChanged = true;
                        }
                    }

                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        PeakCurrentRegister,
                        homeCurrentValue,
                        cancellationToken).ConfigureAwait(false);
                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        StandbyCurrentRegister,
                        standbyValue,
                        cancellationToken).ConfigureAwait(false);

                    await WriteMultipleRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeFastSpeedRegister,
                        homeValues,
                        cancellationToken).ConfigureAwait(false);

                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        Pr0ModeRegister,
                        Pr0AbsoluteInterruptMode,
                        cancellationToken).ConfigureAwait(false);
                    await WriteMultipleRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        Pr0SpeedRegister,
                        autoValues,
                        cancellationToken).ConfigureAwait(false);

                    await SaveAndVerifyAsync(
                        address.Line,
                        slaveId,
                        CommandSaveParameters,
                        cancellationToken).ConfigureAwait(false);

                    if (mappingChanged)
                    {
                        await SaveAndVerifyAsync(
                            address.Line,
                            slaveId,
                            CommandSaveMappings,
                            cancellationToken).ConfigureAwait(false);
                        restartRequired++;
                    }

                    var currentReadBack = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        PeakCurrentRegister,
                        cancellationToken).ConfigureAwait(false);
                    var standbyReadBack = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        StandbyCurrentRegister,
                        cancellationToken).ConfigureAwait(false);
                    var homeReadBack = await ReadRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeFastSpeedRegister,
                        4,
                        cancellationToken).ConfigureAwait(false);
                    var autoReadBack = await ReadRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        Pr0SpeedRegister,
                        4,
                        cancellationToken).ConfigureAwait(false);

                    if (currentReadBack != homeCurrentValue ||
                        standbyReadBack != standbyValue ||
                        !homeReadBack.SequenceEqual(homeValues) ||
                        !autoReadBack.SequenceEqual(autoValues))
                    {
                        throw new InvalidOperationException(
                            $"Đọc kiểm tra không khớp. " +
                            $"Current={currentReadBack}/{homeCurrentValue}, " +
                            $"Standby={standbyReadBack}/{standbyValue}, " +
                            $"HOME={string.Join(",", homeReadBack)}/" +
                            $"{string.Join(",", homeValues)}, " +
                            $"AUTO={string.Join(",", autoReadBack)}/" +
                            $"{string.Join(",", autoValues)}.");
                    }

                    lock (_configSync)
                    {
                        _axisModeSettings[address] = normalized;
                    }

                    success++;
                    axis.LastCommand = mappingChanged
                        ? "COMPLETE_SETTING_SAVED_RESTART_REQUIRED"
                        : "COMPLETE_SETTING_SAVED_VERIFIED";

                    _state.WriteLog(
                        LogLevel.Ok,
                        $"[SETTING COMPLETE] {address.DisplayId}: " +
                        $"HOME={normalized.HomeCurrentAmps:0.0}A, " +
                        $"MANUAL={normalized.ManualCurrentAmps:0.0}A, " +
                        $"AUTO={normalized.AutoCurrentAmps:0.0}A, " +
                        $"PPR={normalized.PulsesPerRevolution:N0}, " +
                        $"HomeFast={normalized.HomeFastSpeedRpm}rpm, " +
                        $"HomeSlow={normalized.HomeSlowSpeedRpm}rpm — OK.");
                }
                catch (Exception ex)
                {
                    failed++;
                    axis.LastCommand = "COMPLETE_SETTING_SAVE_ERROR";
                    _state.WriteLog(
                        LogLevel.Error,
                        $"[SETTING COMPLETE] {address.DisplayId}: {ex.Message}");
                }
            }

            await SaveModeSettingsToDiskAsync(cancellationToken)
                .ConfigureAwait(false);
        }
        finally
        {
            ResumePollingLines(pausedLines);
        }

        _state.NotifyStateChanged();
        _state.WriteLog(
            failed == 0 ? LogLevel.Ok : LogLevel.Warning,
            $"[SETTING COMPLETE] Hoàn tất: thành công={success}, " +
            $"offline={offline}, lỗi={failed}, cần restart mapping={restartRequired}.");
    }

    public async Task SaveModeDriverSettingsAsync(
        IEnumerable<AxisAddress> axes,
        DriverModeSettings settings,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(axes);
        ArgumentNullException.ThrowIfNull(settings);

        var targets = axes.Distinct().ToArray();
        if (targets.Length == 0)
        {
            return;
        }

        var normalized = NormalizeModeSettings(settings);

        lock (_configSync)
        {
            foreach (var address in targets)
            {
                _axisModeSettings[address] = normalized;
            }
        }

        await SaveModeSettingsToDiskAsync(cancellationToken).ConfigureAwait(false);

        var pausedLines = await PausePollingForTargetsAsync(targets).ConfigureAwait(false);
        var success = 0;
        var failed = 0;
        var offline = 0;
        var homeCurrentValue = CurrentAmpsToRegister(normalized.HomeCurrentAmps);
        var homeMotionValues = new ushort[]
        {
            checked((ushort)normalized.HomeFastSpeedRpm),
            checked((ushort)normalized.HomeSlowSpeedRpm),
            checked((ushort)normalized.HomeAccelerationMsPer1000Rpm),
            checked((ushort)normalized.HomeDecelerationMsPer1000Rpm)
        };

        try
        {
            foreach (var address in targets)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var axis = _state.GetAxis(address);

                if (!axis.IsOnline)
                {
                    offline++;
                    continue;
                }

                try
                {
                    await CancelJogAsync(address).ConfigureAwait(false);
                    var slaveId = checked((byte)address.SlaveId);
                    var status = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        MotionStatusRegister,
                        cancellationToken).ConfigureAwait(false);

                    if ((status & 0x0004) != 0)
                    {
                        throw new InvalidOperationException(
                            "Driver đang chạy. Hãy dừng motor trước khi lưu cấu hình chế độ.");
                    }

                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        PeakCurrentRegister,
                        homeCurrentValue,
                        cancellationToken).ConfigureAwait(false);

                    // Bốn thanh ghi HOME liên tục nên ghi bằng một khung FC10,
                    // giảm số lần truyền và giảm khả năng timeout.
                    await WriteMultipleRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeFastSpeedRegister,
                        homeMotionValues,
                        cancellationToken).ConfigureAwait(false);

                    await SaveAndVerifyAsync(
                        address.Line,
                        slaveId,
                        CommandSaveParameters,
                        cancellationToken).ConfigureAwait(false);

                    var currentReadBack = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        PeakCurrentRegister,
                        cancellationToken).ConfigureAwait(false);
                    var homeReadBack = await ReadRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeFastSpeedRegister,
                        4,
                        cancellationToken).ConfigureAwait(false);

                    if (currentReadBack != homeCurrentValue ||
                        !homeReadBack.SequenceEqual(homeMotionValues))
                    {
                        throw new InvalidOperationException(
                            $"Đọc lại không khớp. Current={currentReadBack}/{homeCurrentValue}, " +
                            $"HOME={string.Join(",", homeReadBack)}/" +
                            $"{string.Join(",", homeMotionValues)}.");
                    }

                    success++;
                    axis.LastCommand = "MODE_CURRENT_HOME_PPR_SAVED";
                    _state.WriteLog(
                        LogLevel.Ok,
                        $"[MODE SETTING] {address.DisplayId}: HOME={normalized.HomeCurrentAmps:0.0}A, " +
                        $"Fast={normalized.HomeFastSpeedRpm} rpm, " +
                        $"Slow={normalized.HomeSlowSpeedRpm} rpm, " +
                        $"Acc={normalized.HomeAccelerationMsPer1000Rpm}, " +
                        $"Dec={normalized.HomeDecelerationMsPer1000Rpm} đã ghi và lưu EEPROM; " +
                        $"MANUAL={normalized.ManualCurrentAmps:0.0}A, " +
                        $"AUTO={normalized.AutoCurrentAmps:0.0}A, " +
                        $"PPR={normalized.PulsesPerRevolution:N0} đã lưu phần mềm.");
                }
                catch (Exception ex)
                {
                    failed++;
                    axis.LastCommand = "MODE_SETTING_SAVE_ERROR";
                    _state.WriteLog(
                        LogLevel.Error,
                        $"[MODE SETTING] {address.DisplayId}: {ex.Message}");
                }
            }
        }
        finally
        {
            ResumePollingLines(pausedLines);
        }

        _state.NotifyStateChanged();
        _state.WriteLog(
            failed == 0 ? LogLevel.Ok : LogLevel.Warning,
            $"[MODE SETTING] Hoàn tất: hardware OK={success}, offline={offline}, lỗi={failed}. " +
            $"Giới hạn dòng bắt buộc {MinimumPeakCurrentAmps:0.0}–{MaximumPeakCurrentAmps:0.0}A.");
    }

    public async Task<DriverModeSettings> ReadModeDriverSettingsAsync(
        AxisAddress address,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        var settings = GetModeSettings(address);

        var axis = _state.GetAxis(address);
        if (!axis.IsOnline)
        {
            return settings;
        }

        var pausedLines = await PausePollingForTargetsAsync(new[] { address })
            .ConfigureAwait(false);

        try
        {
            var slaveId = checked((byte)address.SlaveId);
            var activeCurrentRaw = await ReadRegisterCheckedAsync(
                address.Line,
                slaveId,
                PeakCurrentRegister,
                cancellationToken).ConfigureAwait(false);
            var homeValues = await ReadRegistersCheckedAsync(
                address.Line,
                slaveId,
                HomeFastSpeedRegister,
                4,
                cancellationToken).ConfigureAwait(false);

            settings = NormalizeModeSettings(settings with
            {
                HomeFastSpeedRpm = homeValues[0],
                HomeSlowSpeedRpm = homeValues[1],
                HomeAccelerationMsPer1000Rpm = homeValues[2],
                HomeDecelerationMsPer1000Rpm = homeValues[3]
            });

            lock (_configSync)
            {
                _axisModeSettings[address] = settings;
            }

            _state.WriteLog(
                LogLevel.Info,
                $"[MODE SETTING READ] {address.DisplayId}: " +
                $"HOME={settings.HomeCurrentAmps:0.0}A, " +
                $"MANUAL={settings.ManualCurrentAmps:0.0}A, " +
                $"AUTO={settings.AutoCurrentAmps:0.0}A, " +
                $"PPR={settings.PulsesPerRevolution:N0}, " +
                $"Fast={settings.HomeFastSpeedRpm}, Slow={settings.HomeSlowSpeedRpm}, " +
                $"Acc={settings.HomeAccelerationMsPer1000Rpm}, " +
                $"Dec={settings.HomeDecelerationMsPer1000Rpm}; " +
                $"dòng đang hoạt động={activeCurrentRaw / 10.0:0.0}A.");

            return settings;
        }
        finally
        {
            ResumePollingLines(pausedLines);
        }
    }

    public int GetConfiguredPulsesPerRevolution(AxisAddress address) =>
        GetAxisPulsesPerRevolution(address);

    public async Task SaveDriverConfigAsync(
        IEnumerable<AxisAddress> axes,
        int diPinIndex,
        bool activeLowNC,
        double peakCurrentAmps,
        int standbyPercent,
        double homingSpeedRps,
        double autoSpeedRps,
        double autoAccRps2,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(axes);

        if (diPinIndex is < 1 or > 7)
        {
            throw new ArgumentOutOfRangeException(
                nameof(diPinIndex),
                "DI phải từ 1 đến 7.");
        }

        var targets = axes.Distinct().ToArray();
        if (targets.Length == 0)
        {
            return;
        }

        var normalizedHomeCurrent = NormalizeCurrentAmps(peakCurrentAmps);
        var currentValue = CurrentAmpsToRegister(normalizedHomeCurrent);
        var standbyValue = (ushort)Math.Clamp(standbyPercent, 0, 100);
        var homeFastRpm = (ushort)Math.Clamp(
            (int)Math.Round(homingSpeedRps * 60.0),
            1,
            5000);
        var homeSlowRpm = (ushort)Math.Clamp(
            homeFastRpm / 10,
            1,
            homeFastRpm);
        var autoSpeedRpm = (ushort)Math.Clamp(
            (int)Math.Round(autoSpeedRps * 60.0),
            1,
            5000);
        var accelerationTime =
            AccelerationRps2ToMsPer1000Rpm(autoAccRps2);
        var selectedDiValue =
            activeLowNC ? DiFunctionOrgNc : DiFunctionOrgNo;

        var homeValues = new ushort[]
        {
            homeFastRpm,
            homeSlowRpm,
            accelerationTime,
            accelerationTime
        };
        var autoValues = new ushort[]
        {
            autoSpeedRpm,
            accelerationTime,
            accelerationTime,
            0
        };

        var successCount = 0;
        var failedCount = 0;
        var restartRequiredCount = 0;
        var pausedLines =
            await PausePollingForTargetsAsync(targets).ConfigureAwait(false);

        _state.WriteLog(
            LogLevel.Info,
            $"[SETTING] Lưu {targets.Length} driver: DI{diPinIndex}, " +
            $"HomeCurrent={normalizedHomeCurrent:0.0}A, " +
            $"Home={homingSpeedRps:0.###} vòng/s, " +
            $"AUTO={autoSpeedRps:0.###} vòng/s, " +
            $"Acc={autoAccRps2:0.###} vòng/s².");

        try
        {
            foreach (var address in targets)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var axis = _state.GetAxis(address);
                if (!axis.IsOnline)
                {
                    continue;
                }

                try
                {
                    await CancelJogAsync(address).ConfigureAwait(false);
                    var slaveId = checked((byte)address.SlaveId);
                    var status = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        MotionStatusRegister,
                        cancellationToken).ConfigureAwait(false);

                    if ((status & 0x0004) != 0)
                    {
                        throw new InvalidOperationException(
                            "Driver đang chạy. Hãy dừng motor trước khi lưu Setting.");
                    }

                    var mappingChanged = false;

                    // Chỉ xóa chức năng ORG bị gán trùng. Không đụng các DI khác.
                    for (var pin = 1; pin <= 7; pin++)
                    {
                        var register = DigitalInputFunctionRegisters[pin];
                        var existing = await ReadRegisterCheckedAsync(
                            address.Line,
                            slaveId,
                            register,
                            cancellationToken).ConfigureAwait(false);

                        if (pin == diPinIndex)
                        {
                            if (existing != selectedDiValue)
                            {
                                await WriteRegisterCheckedAsync(
                                    address.Line,
                                    slaveId,
                                    register,
                                    selectedDiValue,
                                    cancellationToken).ConfigureAwait(false);
                                mappingChanged = true;
                            }
                        }
                        else if (IsOrgFunction(existing))
                        {
                            await WriteRegisterCheckedAsync(
                                address.Line,
                                slaveId,
                                register,
                                DiFunctionInvalid,
                                cancellationToken).ConfigureAwait(false);
                            mappingChanged = true;
                        }
                    }

                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        PeakCurrentRegister,
                        currentValue,
                        cancellationToken).ConfigureAwait(false);
                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        StandbyCurrentRegister,
                        standbyValue,
                        cancellationToken).ConfigureAwait(false);

                    // 0x600F..0x6012 liên tục: ghi chung một FC10.
                    await WriteMultipleRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeFastSpeedRegister,
                        homeValues,
                        cancellationToken).ConfigureAwait(false);

                    await WriteRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        Pr0ModeRegister,
                        Pr0AbsoluteInterruptMode,
                        cancellationToken).ConfigureAwait(false);

                    // 0x6203..0x6206 liên tục: Speed, Acc, Dec, Pause.
                    await WriteMultipleRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        Pr0SpeedRegister,
                        autoValues,
                        cancellationToken).ConfigureAwait(false);

                    await SaveAndVerifyAsync(
                        address.Line,
                        slaveId,
                        CommandSaveParameters,
                        cancellationToken).ConfigureAwait(false);

                    if (mappingChanged)
                    {
                        await SaveAndVerifyAsync(
                            address.Line,
                            slaveId,
                            CommandSaveMappings,
                            cancellationToken).ConfigureAwait(false);
                        restartRequiredCount++;
                    }

                    var currentReadBack = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        PeakCurrentRegister,
                        cancellationToken).ConfigureAwait(false);
                    var standbyReadBack = await ReadRegisterCheckedAsync(
                        address.Line,
                        slaveId,
                        StandbyCurrentRegister,
                        cancellationToken).ConfigureAwait(false);
                    var homeReadBack = await ReadRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        HomeFastSpeedRegister,
                        4,
                        cancellationToken).ConfigureAwait(false);
                    var autoReadBack = await ReadRegistersCheckedAsync(
                        address.Line,
                        slaveId,
                        Pr0SpeedRegister,
                        4,
                        cancellationToken).ConfigureAwait(false);

                    if (currentReadBack != currentValue ||
                        standbyReadBack != standbyValue ||
                        !homeReadBack.SequenceEqual(homeValues) ||
                        !autoReadBack.SequenceEqual(autoValues))
                    {
                        throw new InvalidOperationException(
                            $"Đọc lại không khớp: Current={currentReadBack}/{currentValue}, " +
                            $"Standby={standbyReadBack}/{standbyValue}, " +
                            $"HOME={string.Join(",", homeReadBack)}/" +
                            $"{string.Join(",", homeValues)}, " +
                            $"AUTO={string.Join(",", autoReadBack)}/" +
                            $"{string.Join(",", autoValues)}.");
                    }

                    successCount++;
                    axis.LastCommand = mappingChanged
                        ? "SETTING_SAVED_RESTART_REQUIRED"
                        : "SETTING_SAVED_VERIFIED";
                    _state.WriteLog(
                        LogLevel.Ok,
                        $"[SETTING] {address.DisplayId}: ghi, lưu EEPROM và " +
                        "đọc kiểm tra thành công.");
                }
                catch (Exception ex)
                {
                    failedCount++;
                    axis.LastCommand = "SETTING_SAVE_ERROR";
                    _state.WriteLog(
                        LogLevel.Error,
                        $"[SETTING] {address.DisplayId}: {ex.Message}");
                }
            }

            lock (_configSync)
            {
                foreach (var address in targets)
                {
                    var existing = GetModeSettingsWithoutLock(address);
                    _axisModeSettings[address] = NormalizeModeSettings(
                        existing with
                        {
                            HomeCurrentAmps = normalizedHomeCurrent,
                            HomeFastSpeedRpm = homeFastRpm,
                            HomeSlowSpeedRpm = homeSlowRpm,
                            HomeAccelerationMsPer1000Rpm = accelerationTime,
                            HomeDecelerationMsPer1000Rpm = accelerationTime
                        });
                }
            }

            try
            {
                await SaveModeSettingsToDiskAsync(cancellationToken)
                    .ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                _state.WriteLog(
                    LogLevel.Warning,
                    $"[SETTING] Driver đã lưu nhưng không lưu được profile " +
                    $"phần mềm: {ex.Message}");
            }
        }
        finally
        {
            ResumePollingLines(pausedLines);
        }

        _state.NotifyStateChanged();
        _state.WriteLog(
            failedCount == 0 ? LogLevel.Ok : LogLevel.Warning,
            $"[SETTING] Hoàn tất: thành công={successCount}, lỗi={failedCount}, " +
            $"cần restart do đổi mapping={restartRequiredCount}.");
    }

    public async Task<(
        int diPinIndex,
        bool activeLowNC,
        double peakCurrentAmps,
        int standbyPercent,
        double homingSpeedRps,
        double autoSpeedRps,
        double autoAccRps2)> ReadDriverConfigAsync(
            AxisAddress address,
            CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        RequireOnline(address);

        var pausedLines = await PausePollingForTargetsAsync(new[] { address })
            .ConfigureAwait(false);

        try
        {
            var slaveId = checked((byte)address.SlaveId);
            var currentRaw = await ReadRegisterCheckedAsync(
                address.Line,
                slaveId,
                PeakCurrentRegister,
                cancellationToken).ConfigureAwait(false);
            var standbyRaw = await ReadRegisterCheckedAsync(
                address.Line,
                slaveId,
                StandbyCurrentRegister,
                cancellationToken).ConfigureAwait(false);
            var homeValues = await ReadRegistersCheckedAsync(
                address.Line,
                slaveId,
                HomeFastSpeedRegister,
                4,
                cancellationToken).ConfigureAwait(false);
            var autoValues = await ReadRegistersCheckedAsync(
                address.Line,
                slaveId,
                Pr0SpeedRegister,
                3,
                cancellationToken).ConfigureAwait(false);

            var detectedDi = 0;
            var isNc = false;

            for (var pin = 1; pin <= 7; pin++)
            {
                var value = await ReadRegisterCheckedAsync(
                    address.Line,
                    slaveId,
                    DigitalInputFunctionRegisters[pin],
                    cancellationToken).ConfigureAwait(false);

                if (IsOrgFunction(value))
                {
                    detectedDi = pin;
                    isNc = (value & 0x0080) != 0;
                    break;
                }
            }

            var peakAmps = currentRaw / 10.0;
            var standbyPercent = (int)standbyRaw;
            var homingRps = homeValues[0] / 60.0;
            var autoRps = autoValues[0] / 60.0;
            var autoAccRps2 = MsPer1000RpmToAccelerationRps2(autoValues[1]);

            _state.WriteLog(
                LogLevel.Info,
                $"[SETTING READ] {address.DisplayId}: " +
                $"DI{detectedDi} {(isNc ? "N.C." : "N.O.")}, " +
                $"Peak={peakAmps:0.0}A, Standby={standbyPercent}%, " +
                $"HomeFast={homeValues[0]} rpm, HomeSlow={homeValues[1]} rpm, " +
                $"HomeAcc={homeValues[2]}, HomeDec={homeValues[3]}, " +
                $"AUTO={autoRps:0.###} vòng/s, Acc={autoAccRps2:0.###} vòng/s².");

            return (
                detectedDi,
                isNc,
                peakAmps,
                standbyPercent,
                homingRps,
                autoRps,
                autoAccRps2);
        }
        finally
        {
            ResumePollingLines(pausedLines);
        }
    }

    public async Task ClearDriverConfigAsync(
        IEnumerable<AxisAddress> axes,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(axes);

        var targets = axes.Distinct().ToArray();
        var success = 0;
        var failed = 0;

        foreach (var address in targets)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var axis = _state.GetAxis(address);
            if (!axis.IsOnline)
            {
                continue;
            }

            try
            {
                await WriteRegisterCheckedAsync(
                    address.Line,
                    checked((byte)address.SlaveId),
                    SaveControlRegister,
                    CommandResetParametersKeepMotor,
                    cancellationToken).ConfigureAwait(false);
                axis.LastCommand = "PARAMETERS_RESET_RESTART_REQUIRED";
                success++;
            }
            catch (Exception ex)
            {
                failed++;
                _state.WriteLog(
                    LogLevel.Error,
                    $"[RESET PARAM] {address.DisplayId}: {ex.Message}");
            }
        }

        _state.NotifyStateChanged();
        _state.WriteLog(
            failed == 0 ? LogLevel.Ok : LogLevel.Warning,
            $"[RESET PARAM] thành công={success}, lỗi={failed}. Cần tắt/bật nguồn driver.");
    }

    public async Task ResetAlarmAsync(
        IEnumerable<AxisAddress> axes,
        CancellationToken cancellationToken = default)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(axes);

        foreach (var address in axes.Distinct())
        {
            cancellationToken.ThrowIfCancellationRequested();
            var axis = _state.GetAxis(address);
            if (!axis.IsOnline)
            {
                continue;
            }

            try
            {
                var slaveId = checked((byte)address.SlaveId);
                var before = await ReadRegisterCheckedAsync(
                    address.Line,
                    slaveId,
                    AlarmRegister,
                    cancellationToken).ConfigureAwait(false);
                var after = await TryResetAlarmAndReadBackAsync(
                    address,
                    before,
                    cancellationToken).ConfigureAwait(false);

                if (after == 0)
                {
                    axis.State = AxisMotionState.Online;
                    axis.AlarmText = string.Empty;
                    axis.LastCommand = "ALARM_RESET_OK";
                    _state.WriteLog(LogLevel.Ok, $"[RESET ALARM] {address.DisplayId}: OK.");
                }
                else
                {
                    axis.State = AxisMotionState.Alarm;
                    axis.AlarmText = DescribeAlarm(after);
                    axis.LastCommand = $"ALARM_0x{after:X4}";
                    _state.WriteLog(
                        LogLevel.Error,
                        $"[RESET ALARM] {address.DisplayId}: còn Alarm 0x{after:X4} — {DescribeAlarm(after)}");
                }
            }
            catch (Exception ex)
            {
                _state.WriteLog(
                    LogLevel.Error,
                    $"[RESET ALARM] {address.DisplayId}: {ex.Message}");
            }
        }

        _state.NotifyStateChanged();
    }

    #endregion

    #region Modbus RTU transport

    private async Task<ushort> ReadRegisterCheckedAsync(
        int line,
        byte slaveId,
        ushort registerAddress,
        CancellationToken cancellationToken)
    {
        var values = await ReadRegistersCheckedAsync(
            line,
            slaveId,
            registerAddress,
            1,
            cancellationToken).ConfigureAwait(false);
        return values[0];
    }

    private async Task<ushort[]> ReadRegistersCheckedAsync(
        int line,
        byte slaveId,
        ushort startAddress,
        ushort count,
        CancellationToken cancellationToken)
    {
        ValidateLine(line);
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);

        try
        {
            var port = await GetOrReconnectPortUnderLockAsync(
                line,
                $"FC03 READ 0x{startAddress:X4}",
                cancellationToken).ConfigureAwait(false);

            Exception? lastError = null;
            for (var attempt = 1; attempt <= ModbusRequestRetryCount; attempt++)
            {
                try
                {
                    var result = await ReadHoldingRegistersOnOpenPortAsync(
                        port,
                        slaveId,
                        startAddress,
                        count,
                        cancellationToken).ConfigureAwait(false);

                    await Task.Delay(InterCommandDelayMs, cancellationToken)
                        .ConfigureAwait(false);
                    return result;
                }
                catch (Exception ex) when (IsTransientModbusException(ex))
                {
                    lastError = ex;
                    SafeDiscardInput(port);

                    if (attempt < ModbusRequestRetryCount)
                    {
                        await Task.Delay(
                            ModbusRetryDelayMs * attempt,
                            cancellationToken).ConfigureAwait(false);
                    }
                }
            }

            throw new IOException(
                $"FC03 READ thất bại: Line {line}, Slave {slaveId}, " +
                $"Reg=0x{startAddress:X4}, Count={count}. {lastError?.Message}",
                lastError);
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    private async Task WriteRegisterCheckedAsync(
        int line,
        byte slaveId,
        ushort registerAddress,
        ushort value,
        CancellationToken cancellationToken)
    {
        ValidateLine(line);
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);

        try
        {
            var port = await GetOrReconnectPortUnderLockAsync(
                line,
                $"FC06 WRITE 0x{registerAddress:X4}",
                cancellationToken).ConfigureAwait(false);

            Exception? lastError = null;
            for (var attempt = 1; attempt <= ModbusRequestRetryCount; attempt++)
            {
                try
                {
                    await WriteSingleRegisterOnOpenPortAsync(
                        port,
                        slaveId,
                        registerAddress,
                        value,
                        cancellationToken).ConfigureAwait(false);

                    await Task.Delay(InterCommandDelayMs, cancellationToken)
                        .ConfigureAwait(false);
                    return;
                }
                catch (Exception ex) when (IsTransientModbusException(ex))
                {
                    lastError = ex;
                    SafeDiscardInput(port);

                    if (attempt < ModbusRequestRetryCount)
                    {
                        await Task.Delay(
                            ModbusRetryDelayMs * attempt,
                            cancellationToken).ConfigureAwait(false);
                    }
                }
            }

            throw new IOException(
                $"FC06 WRITE thất bại: Line {line}, Slave {slaveId}, " +
                $"Reg=0x{registerAddress:X4}, Value=0x{value:X4}. {lastError?.Message}",
                lastError);
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    private async Task WriteMultipleRegistersCheckedAsync(
        int line,
        byte slaveId,
        ushort startAddress,
        IReadOnlyList<ushort> values,
        CancellationToken cancellationToken)
    {
        ValidateLine(line);
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);

        try
        {
            var port = await GetOrReconnectPortUnderLockAsync(
                line,
                $"FC10 WRITE 0x{startAddress:X4}",
                cancellationToken).ConfigureAwait(false);

            Exception? lastError = null;
            for (var attempt = 1; attempt <= ModbusRequestRetryCount; attempt++)
            {
                try
                {
                    await WriteMultipleRegistersOnOpenPortAsync(
                        port,
                        slaveId,
                        startAddress,
                        values,
                        cancellationToken).ConfigureAwait(false);

                    await Task.Delay(InterCommandDelayMs, cancellationToken)
                        .ConfigureAwait(false);
                    return;
                }
                catch (Exception ex) when (IsTransientModbusException(ex))
                {
                    lastError = ex;
                    SafeDiscardInput(port);

                    if (attempt < ModbusRequestRetryCount)
                    {
                        await Task.Delay(
                            ModbusRetryDelayMs * attempt,
                            cancellationToken).ConfigureAwait(false);
                    }
                }
            }

            throw new IOException(
                $"FC10 WRITE thất bại: Line {line}, Slave {slaveId}, " +
                $"StartReg=0x{startAddress:X4}, Count={values.Count}. {lastError?.Message}",
                lastError);
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    private static async Task<ushort> ReadSingleRegisterOnOpenPortAsync(
        SerialPort port,
        byte slaveId,
        ushort registerAddress,
        CancellationToken cancellationToken)
    {
        var values = await ReadHoldingRegistersOnOpenPortAsync(
            port,
            slaveId,
            registerAddress,
            1,
            cancellationToken).ConfigureAwait(false);
        return values[0];
    }

    private static async Task<ushort[]> ReadHoldingRegistersOnOpenPortAsync(
        SerialPort port,
        byte slaveId,
        ushort startAddress,
        ushort count,
        CancellationToken cancellationToken)
    {
        if (count is 0 or > 125)
        {
            throw new ArgumentOutOfRangeException(nameof(count));
        }

        PreparePortForRequest(port);
        var request = BuildReadRequest(slaveId, startAddress, count);
        port.Write(request, 0, request.Length);
        await FlushRequestAsync(port, cancellationToken).ConfigureAwait(false);

        var deadline = DateTime.UtcNow.AddMilliseconds(ModbusTimeoutMs);
        var header = new byte[3];
        await ReadExactAsync(
            port,
            header,
            0,
            header.Length,
            deadline,
            cancellationToken).ConfigureAwait(false);

        if (header[0] != slaveId)
        {
            throw new InvalidDataException(
                $"Sai Slave ID. Cần {slaveId}, nhận {header[0]}.");
        }

        if (header[1] == (0x03 | 0x80))
        {
            var exceptionTail = new byte[2];
            await ReadExactAsync(
                port,
                exceptionTail,
                0,
                exceptionTail.Length,
                deadline,
                cancellationToken).ConfigureAwait(false);
            var exceptionFrame = header.Concat(exceptionTail).ToArray();
            ValidateCrc(exceptionFrame);
            throw new InvalidOperationException(
                $"Modbus Exception FC03, code=0x{header[2]:X2}.");
        }

        if (header[1] != 0x03)
        {
            throw new InvalidDataException($"Sai Function Code 0x{header[1]:X2}.");
        }

        var expectedByteCount = count * 2;
        if (header[2] != expectedByteCount)
        {
            throw new InvalidDataException(
                $"Sai số byte dữ liệu. Cần {expectedByteCount}, nhận {header[2]}.");
        }

        var tail = new byte[header[2] + 2];
        await ReadExactAsync(
            port,
            tail,
            0,
            tail.Length,
            deadline,
            cancellationToken).ConfigureAwait(false);

        var frame = header.Concat(tail).ToArray();
        ValidateCrc(frame);

        var result = new ushort[count];
        for (var index = 0; index < count; index++)
        {
            var offset = 3 + index * 2;
            result[index] = (ushort)((frame[offset] << 8) | frame[offset + 1]);
        }

        return result;
    }

    private static async Task WriteSingleRegisterOnOpenPortAsync(
        SerialPort port,
        byte slaveId,
        ushort registerAddress,
        ushort value,
        CancellationToken cancellationToken)
    {
        PreparePortForRequest(port);
        var request = BuildWriteSingleRequest(slaveId, registerAddress, value);
        port.Write(request, 0, request.Length);
        await FlushRequestAsync(port, cancellationToken).ConfigureAwait(false);

        // Broadcast ID 0 không trả phản hồi.
        if (slaveId == 0)
        {
            await Task.Delay(8, cancellationToken).ConfigureAwait(false);
            return;
        }

        var response = await ReadWriteResponseAsync(
            port,
            slaveId,
            0x06,
            cancellationToken).ConfigureAwait(false);

        if (!request.SequenceEqual(response))
        {
            throw new InvalidDataException(
                $"Driver không echo đúng lệnh ghi 0x{registerAddress:X4}.");
        }
    }

    private static async Task WriteMultipleRegistersOnOpenPortAsync(
        SerialPort port,
        byte slaveId,
        ushort startAddress,
        IReadOnlyList<ushort> values,
        CancellationToken cancellationToken)
    {
        if (values.Count is 0 or > 123)
        {
            throw new ArgumentOutOfRangeException(nameof(values));
        }

        PreparePortForRequest(port);
        var request = BuildWriteMultipleRequest(slaveId, startAddress, values);
        port.Write(request, 0, request.Length);
        await FlushRequestAsync(port, cancellationToken).ConfigureAwait(false);

        if (slaveId == 0)
        {
            await Task.Delay(8, cancellationToken).ConfigureAwait(false);
            return;
        }

        var response = await ReadWriteResponseAsync(
            port,
            slaveId,
            0x10,
            cancellationToken).ConfigureAwait(false);

        var count = (ushort)values.Count;
        var expected = BuildWriteMultipleAcknowledge(slaveId, startAddress, count);
        if (!expected.SequenceEqual(response))
        {
            throw new InvalidDataException(
                $"Driver không xác nhận đúng FC10 tại 0x{startAddress:X4}.");
        }
    }

    private static async Task<byte[]> ReadWriteResponseAsync(
        SerialPort port,
        byte slaveId,
        byte functionCode,
        CancellationToken cancellationToken)
    {
        var deadline = DateTime.UtcNow.AddMilliseconds(ModbusTimeoutMs);
        var firstFive = new byte[5];
        await ReadExactAsync(
            port,
            firstFive,
            0,
            firstFive.Length,
            deadline,
            cancellationToken).ConfigureAwait(false);

        if (firstFive[0] != slaveId)
        {
            throw new InvalidDataException(
                $"Sai Slave ID. Cần {slaveId}, nhận {firstFive[0]}.");
        }

        if (firstFive[1] == (functionCode | 0x80))
        {
            ValidateCrc(firstFive);
            throw new InvalidOperationException(
                $"Modbus Exception FC{functionCode:X2}, code=0x{firstFive[2]:X2}.");
        }

        if (firstFive[1] != functionCode)
        {
            throw new InvalidDataException(
                $"Sai Function Code 0x{firstFive[1]:X2}, cần 0x{functionCode:X2}.");
        }

        var remaining = new byte[3];
        await ReadExactAsync(
            port,
            remaining,
            0,
            remaining.Length,
            deadline,
            cancellationToken).ConfigureAwait(false);

        var frame = firstFive.Concat(remaining).ToArray();
        ValidateCrc(frame);
        return frame;
    }

    private static async Task FlushRequestAsync(
        SerialPort port,
        CancellationToken cancellationToken)
    {
        try
        {
            await port.BaseStream.FlushAsync(cancellationToken).ConfigureAwait(false);
        }
        catch (NotSupportedException)
        {
            // Một số driver SerialPort không hỗ trợ FlushAsync.
        }

        // Cho bộ chuyển USB-RS485 đủ thời gian chuyển TX -> RX.
        await Task.Delay(FrameTurnaroundDelayMs, cancellationToken).ConfigureAwait(false);
    }

    private static SerialPort CreateSerialPort(string portName, int baudRate)
    {
        return new SerialPort(
            portName,
            baudRate,
            Parity.None,
            8,
            StopBits.One)
        {
            ReadTimeout = ModbusTimeoutMs,
            WriteTimeout = ModbusTimeoutMs,
            Handshake = Handshake.None,
            DtrEnable = false,
            RtsEnable = false
        };
    }

    private bool IsLinePortOpen(int line)
    {
        if (!_ports.TryGetValue(line, out var port))
        {
            return false;
        }

        try
        {
            return port.IsOpen;
        }
        catch
        {
            return false;
        }
    }

    private async Task<bool> EnsureLineAvailableForPollingAsync(
        int line,
        CancellationToken cancellationToken)
    {
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (IsLinePortOpen(line))
            {
                return true;
            }

            var port = await TryReconnectLineUnderLockAsync(
                line,
                "POLL WATCHDOG",
                cancellationToken,
                respectReconnectCooldown: true).ConfigureAwait(false);

            return port is not null;
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    /// <summary>
    /// Trả về port đang mở. Hàm này chỉ được gọi khi caller đã giữ _lineLocks[line].
    /// Nếu USB-RS485 bị reset, hàm sẽ thử mở lại đúng COM/baud đã lưu.
    /// </summary>
    private async Task<SerialPort> GetOrReconnectPortUnderLockAsync(
        int line,
        string operation,
        CancellationToken cancellationToken,
        bool respectReconnectCooldown = false)
    {
        if (_ports.TryGetValue(line, out var existing))
        {
            try
            {
                if (existing.IsOpen)
                {
                    return existing;
                }
            }
            catch
            {
                // Port object đã lỗi; thực hiện mở lại bên dưới.
            }
        }

        var reopened = await TryReconnectLineUnderLockAsync(
            line,
            operation,
            cancellationToken,
            respectReconnectCooldown).ConfigureAwait(false);

        if (reopened is not null)
        {
            return reopened;
        }

        throw new InvalidOperationException(
            $"Line {line} mất kết nối cổng COM và tự kết nối lại thất bại.");
    }

    /// <summary>
    /// Caller phải giữ _lineLocks[line]. Không quét 16 Slave khi reconnect để
    /// Quick Stop có thể được gửi nhanh; polling sẽ xác nhận từng driver sau đó.
    /// </summary>
    private async Task<SerialPort?> TryReconnectLineUnderLockAsync(
        int line,
        string reason,
        CancellationToken cancellationToken,
        bool respectReconnectCooldown)
    {
        ValidateLine(line);

        if (IsLinePortOpen(line) &&
            _ports.TryGetValue(line, out var alreadyOpen))
        {
            return alreadyOpen;
        }

        if (respectReconnectCooldown &&
            _lastReconnectAttemptUtc.TryGetValue(line, out var lastAttempt) &&
            DateTime.UtcNow - lastAttempt <
            TimeSpan.FromMilliseconds(PortReconnectCooldownMs))
        {
            return null;
        }

        _lastReconnectAttemptUtc[line] = DateTime.UtcNow;

        if (!_lineSerialSettings.TryGetValue(line, out var settings))
        {
            var connection = _state.Lines[line - 1];

            // Khi người dùng đã bấm Disconnect, IsConnected=false và cấu hình
            // reconnect đã bị xóa. Không được tự mở lại ngoài ý muốn.
            if (!connection.IsConnected ||
                string.IsNullOrWhiteSpace(connection.PortName) ||
                connection.BaudRate <= 0)
            {
                return null;
            }

            settings = new LineSerialSettings(
                connection.PortName.Trim(),
                connection.BaudRate);
            _lineSerialSettings[line] = settings;
        }

        ClosePortWithoutLock(line);

        Exception? lastError = null;
        for (var attempt = 1; attempt <= PortReconnectAttempts; attempt++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            SerialPort? reopened = null;

            try
            {
                reopened = CreateSerialPort(
                    settings.PortName,
                    settings.BaudRate);
                reopened.Open();

                await Task.Delay(
                    PortOpenStabilizationMs,
                    cancellationToken).ConfigureAwait(false);

                reopened.DiscardInBuffer();
                reopened.DiscardOutBuffer();
                _ports[line] = reopened;

                var connection = _state.Lines[line - 1];
                connection.PortName = settings.PortName;
                connection.BaudRate = settings.BaudRate;
                connection.IsConnected = true;

                _lineReconnectFailures[line] = 0;
                _state.NotifyStateChanged();
                _state.WriteLog(
                    LogLevel.Ok,
                    $"[RS485 RECONNECT] Line {line}: đã mở lại " +
                    $"{settings.PortName} ({settings.BaudRate} bps), lý do={reason}.");

                return reopened;
            }
            catch (OperationCanceledException)
            {
                try
                {
                    reopened?.Dispose();
                }
                catch
                {
                    // Không che lỗi cancel.
                }
                throw;
            }
            catch (Exception ex)
            {
                lastError = ex;
                try
                {
                    if (reopened?.IsOpen == true)
                    {
                        reopened.Close();
                    }
                    reopened?.Dispose();
                }
                catch
                {
                    // Tiếp tục retry.
                }

                if (attempt < PortReconnectAttempts)
                {
                    await Task.Delay(
                        PortReconnectDelayMs * attempt,
                        cancellationToken).ConfigureAwait(false);
                }
            }
        }

        var stateConnection = _state.Lines[line - 1];
        stateConnection.IsConnected = false;

        var failures = _lineReconnectFailures.AddOrUpdate(
            line,
            1,
            (_, current) => current + 1);

        // Hạn chế spam log khi USB chưa được cắm lại.
        if (failures == 1 || failures % 5 == 0)
        {
            _state.WriteLog(
                LogLevel.Error,
                $"[RS485 RECONNECT] Line {line}: không mở lại được " +
                $"{settings.PortName} sau {PortReconnectAttempts} lần, " +
                $"lý do={reason}. {lastError?.Message}");
        }

        _state.NotifyStateChanged();
        return null;
    }

    private static async Task ReadExactAsync(
        SerialPort port,
        byte[] buffer,
        int offset,
        int count,
        DateTime deadline,
        CancellationToken cancellationToken)
    {
        var received = 0;

        while (received < count)
        {
            cancellationToken.ThrowIfCancellationRequested();

            if (DateTime.UtcNow >= deadline)
            {
                throw new TimeoutException(
                    $"Timeout Modbus: chỉ nhận {received}/{count} byte.");
            }

            var available = port.BytesToRead;
            if (available <= 0)
            {
                await Task.Delay(1, cancellationToken).ConfigureAwait(false);
                continue;
            }

            var read = port.Read(
                buffer,
                offset + received,
                Math.Min(count - received, available));

            if (read <= 0)
            {
                await Task.Delay(1, cancellationToken).ConfigureAwait(false);
                continue;
            }

            received += read;
        }
    }

    private static void PreparePortForRequest(SerialPort port)
    {
        if (!port.IsOpen)
        {
            throw new InvalidOperationException("Cổng COM đang đóng.");
        }

        // Chỉ xóa dữ liệu nhận cũ. Không dùng DiscardOutBuffer ở đây vì một số
        // USB-RS485 có bộ đệm truyền riêng; xóa output buffer có thể làm mất phần
        // cuối của khung đang chờ phát và gây hiện tượng nhận thiếu 3 byte CRC.
        SafeDiscardInput(port);
    }

    private static void SafeDiscardInput(SerialPort port)
    {
        try
        {
            if (port.IsOpen)
            {
                port.DiscardInBuffer();
            }
        }
        catch
        {
            // Chỉ là thao tác dọn buffer trước khi retry.
        }
    }

    private static bool IsTransientModbusException(Exception exception) =>
        exception is TimeoutException or IOException or InvalidDataException;

    private static byte[] BuildReadRequest(
        byte slaveId,
        ushort startAddress,
        ushort count)
    {
        var frame = new byte[8];
        frame[0] = slaveId;
        frame[1] = 0x03;
        frame[2] = (byte)(startAddress >> 8);
        frame[3] = (byte)startAddress;
        frame[4] = (byte)(count >> 8);
        frame[5] = (byte)count;
        AppendCrc(frame, 6);
        return frame;
    }

    private static byte[] BuildWriteSingleRequest(
        byte slaveId,
        ushort registerAddress,
        ushort value)
    {
        var frame = new byte[8];
        frame[0] = slaveId;
        frame[1] = 0x06;
        frame[2] = (byte)(registerAddress >> 8);
        frame[3] = (byte)registerAddress;
        frame[4] = (byte)(value >> 8);
        frame[5] = (byte)value;
        AppendCrc(frame, 6);
        return frame;
    }

    private static byte[] BuildWriteMultipleRequest(
        byte slaveId,
        ushort startAddress,
        IReadOnlyList<ushort> values)
    {
        var frame = new byte[9 + values.Count * 2];
        frame[0] = slaveId;
        frame[1] = 0x10;
        frame[2] = (byte)(startAddress >> 8);
        frame[3] = (byte)startAddress;
        frame[4] = (byte)(values.Count >> 8);
        frame[5] = (byte)values.Count;
        frame[6] = (byte)(values.Count * 2);

        for (var index = 0; index < values.Count; index++)
        {
            frame[7 + index * 2] = (byte)(values[index] >> 8);
            frame[8 + index * 2] = (byte)values[index];
        }

        AppendCrc(frame, frame.Length - 2);
        return frame;
    }

    private static byte[] BuildWriteMultipleAcknowledge(
        byte slaveId,
        ushort startAddress,
        ushort count)
    {
        var frame = new byte[8];
        frame[0] = slaveId;
        frame[1] = 0x10;
        frame[2] = (byte)(startAddress >> 8);
        frame[3] = (byte)startAddress;
        frame[4] = (byte)(count >> 8);
        frame[5] = (byte)count;
        AppendCrc(frame, 6);
        return frame;
    }

    private static void AppendCrc(byte[] frame, int dataLength)
    {
        var crc = CalculateCrc(frame, dataLength);
        frame[dataLength] = (byte)(crc & 0xFF);
        frame[dataLength + 1] = (byte)(crc >> 8);
    }

    private static void ValidateCrc(byte[] frame)
    {
        if (!VerifyCrc(frame, frame.Length))
        {
            throw new InvalidDataException("CRC Modbus không hợp lệ.");
        }
    }

    public static ushort CalculateCrc(byte[] buffer, int length)
    {
        ushort crc = 0xFFFF;

        for (var position = 0; position < length; position++)
        {
            crc ^= buffer[position];

            for (var bit = 0; bit < 8; bit++)
            {
                if ((crc & 0x0001) != 0)
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

    private static bool VerifyCrc(byte[] buffer, int length)
    {
        if (length < 4)
        {
            return false;
        }

        var expected = (ushort)(buffer[length - 2] | (buffer[length - 1] << 8));
        var calculated = CalculateCrc(buffer, length - 2);
        return expected == calculated;
    }

    #endregion

    #region Helpers

    private async Task<ushort> EnsureNonZeroRegisterAsync(
        int line,
        byte slaveId,
        ushort register,
        ushort fallback,
        CancellationToken cancellationToken)
    {
        var value = await ReadRegisterCheckedAsync(
            line,
            slaveId,
            register,
            cancellationToken).ConfigureAwait(false);

        if (value != 0)
        {
            return value;
        }

        await WriteRegisterCheckedAsync(
            line,
            slaveId,
            register,
            fallback,
            cancellationToken).ConfigureAwait(false);
        return fallback;
    }

    private async Task EnsureNoAlarmAsync(
        AxisAddress address,
        CancellationToken cancellationToken)
    {
        var alarm = await ReadRegisterCheckedAsync(
            address.Line,
            checked((byte)address.SlaveId),
            AlarmRegister,
            cancellationToken).ConfigureAwait(false);

        if (alarm != 0)
        {
            throw new InvalidOperationException(
                $"Driver {address.DisplayId} Alarm 0x{alarm:X4}: {DescribeAlarm(alarm)}");
        }
    }

    private async Task<ushort> TryResetAlarmAndReadBackAsync(
        AxisAddress address,
        ushort currentAlarm,
        CancellationToken cancellationToken)
    {
        if (currentAlarm == 0)
        {
            return 0;
        }

        // Manual ghi lỗi quá dòng 0x0001 không thể xóa bằng lệnh mềm khi
        // nguyên nhân phần cứng còn tồn tại; tránh Force Enable lại liên tục.
        await WriteRegisterCheckedAsync(
            address.Line,
            checked((byte)address.SlaveId),
            SaveControlRegister,
            CommandResetCurrentAlarm,
            cancellationToken).ConfigureAwait(false);
        await Task.Delay(200, cancellationToken).ConfigureAwait(false);

        return await ReadRegisterCheckedAsync(
            address.Line,
            checked((byte)address.SlaveId),
            AlarmRegister,
            cancellationToken).ConfigureAwait(false);
    }

    private async Task SaveAndVerifyAsync(
        int line,
        byte slaveId,
        ushort saveCommand,
        CancellationToken cancellationToken)
    {
        await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);

        try
        {
            var port = await GetOrReconnectPortUnderLockAsync(
                line,
                $"EEPROM 0x{saveCommand:X4}",
                cancellationToken).ConfigureAwait(false);

            await SaveAndVerifyOnOpenPortAsync(
                port,
                slaveId,
                saveCommand,
                cancellationToken).ConfigureAwait(false);
        }
        finally
        {
            _lineLocks[line].Release();
        }
    }

    private async Task SaveAndVerifyOnOpenPortAsync(
        SerialPort port,
        byte slaveId,
        ushort saveCommand,
        CancellationToken cancellationToken)
    {
        Exception? lastWriteError = null;
        var commandEchoReceived = false;

        // Lệnh lưu EEPROM có thể làm firmware bận ngay sau khi nhận khung.
        for (var attempt = 1; attempt <= ModbusRequestRetryCount; attempt++)
        {
            try
            {
                await WriteSingleRegisterOnOpenPortAsync(
                    port,
                    slaveId,
                    SaveControlRegister,
                    saveCommand,
                    cancellationToken).ConfigureAwait(false);
                commandEchoReceived = true;
                break;
            }
            catch (Exception ex) when (IsTransientModbusException(ex))
            {
                lastWriteError = ex;
                SafeDiscardInput(port);

                if (attempt < ModbusRequestRetryCount)
                {
                    await Task.Delay(
                        ModbusRetryDelayMs * attempt,
                        cancellationToken).ConfigureAwait(false);
                }
            }
        }

        if (!commandEchoReceived)
        {
            throw new IOException(
                $"Không nhận được echo FC06 khi ghi lệnh lưu 0x{saveCommand:X4} " +
                $"vào 0x{SaveControlRegister:X4}, Slave {slaveId}. " +
                $"{lastWriteError?.Message}",
                lastWriteError);
        }

        // Mapping được xác nhận lại bằng cách đọc DI sau khi driver restart.
        if (saveCommand == CommandSaveMappings)
        {
            await Task.Delay(500, cancellationToken).ConfigureAwait(false);
            return;
        }

        Exception? lastStatusError = null;
        var transientFailures = 0;

        // Manual ghi rõ 0x1901 chỉ trả 0x5555 ở lần đọc đầu tiên, sau đó trở
        // về 0x1111. Nếu phản hồi 0x5555 bị mất trên đường RS485 thì lần đọc
        // tiếp theo có thể đã là 0x1111. Khi đó không được báo lỗi giả; phần
        // gọi phía ngoài vẫn đọc lại toàn bộ tham số để xác nhận.
        for (var attempt = 1; attempt <= 10; attempt++)
        {
            await Task.Delay(
                SaveStatusPollDelayMs,
                cancellationToken).ConfigureAwait(false);

            try
            {
                var status = await ReadSingleRegisterOnOpenPortAsync(
                    port,
                    slaveId,
                    SaveStatusRegister,
                    cancellationToken).ConfigureAwait(false);

                if (status == 0x5555)
                {
                    return;
                }

                if (status == 0xAAAA)
                {
                    throw new InvalidOperationException(
                        "Driver báo lưu EEPROM thất bại (0xAAAA).");
                }

                if (status == 0x1111)
                {
                    _state.WriteLog(
                        LogLevel.Warning,
                        $"[EEPROM] Slave {slaveId}: 0x1901 đã trở về 0x1111. " +
                        "Tiếp tục xác nhận bằng đọc lại tham số.");
                    return;
                }
            }
            catch (InvalidOperationException)
            {
                throw;
            }
            catch (Exception ex) when (IsTransientModbusException(ex))
            {
                transientFailures++;
                lastStatusError = ex;
                SafeDiscardInput(port);
            }
        }

        // Echo lệnh lưu đã nhận được. Không dừng toàn bộ thao tác chỉ vì
        // thanh ghi trạng thái bị mất phản hồi; bước readback ngay sau đây
        // sẽ xác nhận các giá trị thực tế.
        _state.WriteLog(
            LogLevel.Warning,
            $"[EEPROM] Slave {slaveId}: không đọc được 0x1901 sau " +
            $"{transientFailures} lỗi tạm thời. Tiếp tục xác nhận bằng readback. " +
            $"Lỗi cuối: {lastStatusError?.Message}");
    }

    private async Task BroadcastQuickStopAsync(CancellationToken cancellationToken)
    {
        // Không chỉ lấy các port đang IsOpen. Nếu USB-RS485 vừa reset thì IsOpen
        // có thể false; Quick Stop phải thử mở lại đúng COM/baud rồi mới gửi.
        var lines = Enumerable.Range(1, 4)
            .Where(line =>
                _lineSerialSettings.ContainsKey(line) ||
                _ports.ContainsKey(line) ||
                _state.Lines[line - 1].IsConnected)
            .ToArray();

        if (lines.Length == 0)
        {
            _state.WriteLog(
                LogLevel.Error,
                "[QUICK STOP] Không có Line nào đã được cấu hình COM. " +
                "Không thể gửi lệnh dừng xuống phần cứng.");
            return;
        }

        var results = await Task.WhenAll(lines.Select(async line =>
        {
            await _lineLocks[line].WaitAsync(cancellationToken).ConfigureAwait(false);
            try
            {
                var port = await GetOrReconnectPortUnderLockAsync(
                    line,
                    "QUICK STOP",
                    cancellationToken).ConfigureAwait(false);

                // Slave 0 là broadcast nên driver không trả phản hồi. Lệnh được gửi
                // tới toàn bộ driver trên Line ngay cả khi trạng thái từng trục Offline.
                try
                {
                    await WriteSingleRegisterOnOpenPortAsync(
                        port,
                        0,
                        PrControlRegister,
                        CommandQuickStop,
                        cancellationToken).ConfigureAwait(false);
                }
                catch (Exception ex) when (IsTransientModbusException(ex) ||
                                           ex is InvalidOperationException ||
                                           ex is UnauthorizedAccessException)
                {
                    // Có trường hợp SerialPort.IsOpen vẫn true nhưng USB-RS485 đã
                    // treo. Tái tạo port và thử Quick Stop thêm một lần.
                    ClosePortWithoutLock(line);
                    _state.Lines[line - 1].IsConnected = false;

                    port = await GetOrReconnectPortUnderLockAsync(
                        line,
                        "QUICK STOP RETRY",
                        cancellationToken).ConfigureAwait(false);

                    await WriteSingleRegisterOnOpenPortAsync(
                        port,
                        0,
                        PrControlRegister,
                        CommandQuickStop,
                        cancellationToken).ConfigureAwait(false);
                }

                _state.WriteLog(
                    LogLevel.Ok,
                    $"[QUICK STOP] Line {line}: đã gửi broadcast 0x6002=0x0040.");
                return true;
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception ex)
            {
                _state.WriteLog(
                    LogLevel.Error,
                    $"[QUICK STOP] Line {line}: không gửi được lệnh dừng — {ex.Message}");
                return false;
            }
            finally
            {
                _lineLocks[line].Release();
            }
        })).ConfigureAwait(false);

        var success = results.Count(value => value);
        var failed = results.Length - success;
        _state.WriteLog(
            failed == 0 ? LogLevel.Ok : LogLevel.Warning,
            $"[QUICK STOP] Hoàn tất: gửi được {success}/{results.Length} Line, lỗi={failed}.");
    }

    private async Task CancelJogAsync(AxisAddress address)
    {
        CancellationTokenSource? cts = null;
        Task? task = null;

        lock (_jogSync)
        {
            if (_jogCts.Remove(address, out var foundCts))
            {
                cts = foundCts;
            }

            if (_jogTasks.Remove(address, out var foundTask))
            {
                task = foundTask;
            }
        }

        if (cts is null)
        {
            return;
        }

        cts.Cancel();
        if (task is not null)
        {
            try
            {
                await task.ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                // Dừng bình thường.
            }
        }
        cts.Dispose();
    }

    private async Task CancelAllJogAsync()
    {
        AxisAddress[] addresses;
        lock (_jogSync)
        {
            addresses = _jogCts.Keys.ToArray();
        }

        foreach (var address in addresses)
        {
            await CancelJogAsync(address).ConfigureAwait(false);
        }
    }

    private async Task CancelAutoWorkerAsync()
    {
        CancellationTokenSource? cts;
        CancellationTokenSource? lidarTransition;
        Task? task;

        lock (_autoSync)
        {
            cts = _autoCts;
            task = _autoTask;
            lidarTransition = _lidarTransitionCts;
            _autoCts = null;
            _autoTask = null;
            _lidarTransitionCts = null;
            _autoPaused = false;
        }

        if (lidarTransition is not null)
        {
            try { lidarTransition.Cancel(); } catch { }
        }

        if (cts is null)
        {
            return;
        }

        cts.Cancel();
        if (task is not null)
        {
            try
            {
                await task.ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                // Dừng bình thường.
            }
        }
        cts.Dispose();
    }

    private static ushort[] BuildPr0Command(
        ushort mode,
        int targetPulses,
        ushort speedRpm,
        ushort acceleration,
        ushort deceleration)
    {
        return new[]
        {
            mode,
            (ushort)((targetPulses >> 16) & 0xFFFF),
            (ushort)(targetPulses & 0xFFFF),
            speedRpm,
            acceleration,
            deceleration,
            (ushort)0,
            CommandTriggerPr0
        };
    }

    private static void ValidateAutoProgram(AutoProgram program)
    {
        if (program.GridRows != AutoProgram.GridSize || program.GridColumns != AutoProgram.GridSize)
            throw new ArgumentException("AUTO phải dùng Grid 16×16.", nameof(program));
        if (program.Clusters.Count == 0)
            throw new ArgumentException("AUTO phải có ít nhất một cụm.", nameof(program));
        if (program.PulsesPerRevolution <= 0)
            throw new ArgumentOutOfRangeException(nameof(program), "Pulse/vòng phải lớn hơn 0.");
        if (program.Clusters.Any(c => c.Width is < 1 or > 16 || c.Height is < 1 or > 16))
            throw new ArgumentException("Kích thước cụm không hợp lệ.", nameof(program));
        if (program.Clusters.Any(c => c.TopRow < 0 || c.LeftColumn < 0 || c.TopRow + c.Height > 16 || c.LeftColumn + c.Width > 16))
            throw new ArgumentException("Cụm vượt khỏi Grid 16×16.", nameof(program));
        if (program.Clusters.Any(c => c.FrequencyHz is < 0.01 or > 5.0 || c.LayerOffsetRevolutions < 0 || c.LayerOffsetRevolutions > 1))
            throw new ArgumentException("Tốc độ motor hoặc lệch lớp không hợp lệ.", nameof(program));

        var drivers = new HashSet<AxisAddress>();
        foreach (var cluster in program.Clusters)
        {
            if (cluster.Cells.Count != cluster.Width * cluster.Height)
            {
                throw new ArgumentException(
                    $"Cụm {cluster.Id} không đủ {cluster.Width * cluster.Height} ô.",
                    nameof(program));
            }

            var missing = cluster.Cells.Where(cell => cell.DriverId is null).ToArray();
            if (missing.Length > 0)
            {
                throw new ArgumentException(
                    $"Cụm {cluster.Id} còn {missing.Length} ô chưa gán Driver ID.",
                    nameof(program));
            }

            foreach (var cell in cluster.Cells)
            {
                var driver = cell.DriverId!.Value;
                if (!drivers.Add(driver))
                    throw new ArgumentException($"Driver {driver.DisplayId} bị gán trùng trong AUTO.", nameof(program));
            }
        }
    }

    #endregion

    private static ushort AccelerationRps2ToMsPer1000Rpm(double accelerationRps2)
    {
        if (accelerationRps2 <= 0)
        {
            return 100;
        }

        // 1000 rpm = 16.6666667 vòng/s.
        var milliseconds = 16_666.6666667 / accelerationRps2;
        return (ushort)Math.Clamp((int)Math.Round(milliseconds), 1, 10_000);
    }

    private static double MsPer1000RpmToAccelerationRps2(ushort milliseconds)
    {
        return milliseconds == 0 ? 0.0 : 16_666.6666667 / milliseconds;
    }

    private static bool IsOrgFunction(ushort value) =>
        (value & 0x007F) == DiFunctionOrgNo;

    private async Task<int[]> PausePollingForTargetsAsync(
        IEnumerable<AxisAddress> targets)
    {
        var activeLines = targets
            .Select(address => address.Line)
            .Distinct()
            .Where(line => _pollCts.ContainsKey(line))
            .ToArray();

        foreach (var line in activeLines)
        {
            await StopLinePollingAsync(line).ConfigureAwait(false);
        }

        return activeLines;
    }

    private void ResumePollingLines(IEnumerable<int> lines)
    {
        foreach (var line in lines.Distinct())
        {
            if (_lineSerialSettings.ContainsKey(line) || IsLinePortOpen(line))
            {
                StartLinePolling(line);
            }
        }
    }

    private async Task<(
        ushort FastSpeedRpm,
        ushort SlowSpeedRpm,
        ushort AccelerationMsPer1000Rpm,
        ushort DecelerationMsPer1000Rpm)> ApplyConfiguredHomeMotionAsync(
            AxisAddress address,
            CancellationToken cancellationToken)
    {
        var settings = GetModeSettings(address);
        var expected = new ushort[]
        {
            checked((ushort)settings.HomeFastSpeedRpm),
            checked((ushort)settings.HomeSlowSpeedRpm),
            checked((ushort)settings.HomeAccelerationMsPer1000Rpm),
            checked((ushort)settings.HomeDecelerationMsPer1000Rpm)
        };
        var slaveId = checked((byte)address.SlaveId);

        var current = await ReadRegistersCheckedAsync(
            address.Line,
            slaveId,
            HomeFastSpeedRegister,
            4,
            cancellationToken).ConfigureAwait(false);

        if (!current.SequenceEqual(expected))
        {
            await WriteMultipleRegistersCheckedAsync(
                address.Line,
                slaveId,
                HomeFastSpeedRegister,
                expected,
                cancellationToken).ConfigureAwait(false);

            current = await ReadRegistersCheckedAsync(
                address.Line,
                slaveId,
                HomeFastSpeedRegister,
                4,
                cancellationToken).ConfigureAwait(false);
        }

        if (!current.SequenceEqual(expected))
        {
            throw new InvalidOperationException(
                $"Không áp dụng được profile HOME: đọc lại " +
                $"{string.Join(",", current)}, yêu cầu {string.Join(",", expected)}.");
        }

        return (current[0], current[1], current[2], current[3]);
    }

    private async Task<double> ApplyConfiguredCurrentAsync(
        AxisAddress address,
        DriverOperatingMode mode,
        CancellationToken cancellationToken)
    {
        var settings = GetModeSettings(address);
        var requestedCurrent = mode switch
        {
            DriverOperatingMode.Home => settings.HomeCurrentAmps,
            DriverOperatingMode.Manual => settings.ManualCurrentAmps,
            DriverOperatingMode.Auto => settings.AutoCurrentAmps,
            _ => settings.HomeCurrentAmps
        };

        return await ApplyRequestedCurrentAsync(
            address,
            requestedCurrent,
            mode.ToString(),
            cancellationToken).ConfigureAwait(false);
    }

    private async Task<double> ApplyRequestedCurrentAsync(
        AxisAddress address,
        double requestedCurrent,
        string modeName,
        CancellationToken cancellationToken)
    {
        var normalizedCurrent = NormalizeCurrentAmps(requestedCurrent);
        var registerValue = CurrentAmpsToRegister(normalizedCurrent);
        var slaveId = checked((byte)address.SlaveId);

        var current = await ReadRegisterCheckedAsync(
            address.Line,
            slaveId,
            PeakCurrentRegister,
            cancellationToken).ConfigureAwait(false);

        if (current != registerValue)
        {
            await WriteRegisterCheckedAsync(
                address.Line,
                slaveId,
                PeakCurrentRegister,
                registerValue,
                cancellationToken).ConfigureAwait(false);

            current = await ReadRegisterCheckedAsync(
                address.Line,
                slaveId,
                PeakCurrentRegister,
                cancellationToken).ConfigureAwait(false);
        }

        if (current != registerValue)
        {
            throw new InvalidOperationException(
                $"Không áp dụng được dòng {modeName}: đọc lại {current / 10.0:0.0}A, " +
                $"yêu cầu {normalizedCurrent:0.0}A.");
        }

        return normalizedCurrent;
    }

    private static double NormalizeCurrentAmps(double currentAmps) =>
        Math.Clamp(
            double.IsFinite(currentAmps) ? currentAmps : MinimumPeakCurrentAmps,
            MinimumPeakCurrentAmps,
            MaximumPeakCurrentAmps);

    private static ushort CurrentAmpsToRegister(double currentAmps) =>
        (ushort)Math.Clamp(
            (int)Math.Round(NormalizeCurrentAmps(currentAmps) * 10.0),
            (int)Math.Round(MinimumPeakCurrentAmps * 10.0),
            (int)Math.Round(MaximumPeakCurrentAmps * 10.0));

    private static DriverModeSettings NormalizeModeSettings(
        DriverModeSettings settings)
    {
        var fast = Math.Clamp(settings.HomeFastSpeedRpm, 1, 5000);
        var slow = Math.Clamp(settings.HomeSlowSpeedRpm, 1, fast);

        return new DriverModeSettings(
            NormalizeCurrentAmps(settings.HomeCurrentAmps),
            NormalizeCurrentAmps(settings.ManualCurrentAmps),
            NormalizeCurrentAmps(settings.AutoCurrentAmps),
            Math.Clamp(settings.PulsesPerRevolution, 200, 51_200))
        {
            HomeFastSpeedRpm = fast,
            HomeSlowSpeedRpm = slow,
            HomeAccelerationMsPer1000Rpm = Math.Clamp(
                settings.HomeAccelerationMsPer1000Rpm,
                1,
                10_000),
            HomeDecelerationMsPer1000Rpm = Math.Clamp(
                settings.HomeDecelerationMsPer1000Rpm,
                1,
                10_000)
        };
    }

    private DriverModeSettings GetModeSettings(AxisAddress address)
    {
        lock (_configSync)
        {
            return GetModeSettingsWithoutLock(address);
        }
    }

    private DriverModeSettings GetModeSettingsWithoutLock(AxisAddress address)
    {
        return _axisModeSettings.TryGetValue(address, out var settings)
            ? NormalizeModeSettings(settings)
            : DriverModeSettings.Default;
    }

    private void SetAxisPulsesPerRevolution(
        AxisAddress address,
        int pulsesPerRevolution)
    {
        lock (_configSync)
        {
            var existing = GetModeSettingsWithoutLock(address);
            _axisModeSettings[address] = NormalizeModeSettings(
                existing with { PulsesPerRevolution = pulsesPerRevolution });
        }
    }

    private int GetAxisPulsesPerRevolution(AxisAddress address) =>
        GetModeSettings(address).PulsesPerRevolution;

    private void LoadModeSettingsFromDisk()
    {
        try
        {
            if (!File.Exists(_modeSettingsFilePath))
            {
                return;
            }

            var json = File.ReadAllText(_modeSettingsFilePath);
            var saved = JsonSerializer.Deserialize<
                Dictionary<string, DriverModeSettings>>(json);

            if (saved is null)
            {
                return;
            }

            lock (_configSync)
            {
                foreach (var pair in saved)
                {
                    if (AxisAddress.TryParse(pair.Key, out var address))
                    {
                        _axisModeSettings[address] =
                            NormalizeModeSettings(pair.Value);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[MODE SETTING] Không đọc được file cấu hình phần mềm: {ex.Message}");
        }
    }

    private async Task SaveModeSettingsToDiskAsync(
        CancellationToken cancellationToken)
    {
        await _modeSettingsFileLock
            .WaitAsync(cancellationToken)
            .ConfigureAwait(false);

        try
        {
            Dictionary<string, DriverModeSettings> snapshot;
            lock (_configSync)
            {
                snapshot = _axisModeSettings.ToDictionary(
                    pair => pair.Key.DisplayId,
                    pair => NormalizeModeSettings(pair.Value),
                    StringComparer.OrdinalIgnoreCase);
            }

            var directory = Path.GetDirectoryName(_modeSettingsFilePath);
            if (!string.IsNullOrWhiteSpace(directory))
            {
                Directory.CreateDirectory(directory);
            }

            var json = JsonSerializer.Serialize(
                snapshot,
                new JsonSerializerOptions { WriteIndented = true });
            var temporaryPath = _modeSettingsFilePath + ".tmp";

            await File.WriteAllTextAsync(
                temporaryPath,
                json,
                cancellationToken).ConfigureAwait(false);

            File.Move(
                temporaryPath,
                _modeSettingsFilePath,
                overwrite: true);
        }
        finally
        {
            _modeSettingsFileLock.Release();
        }
    }

    private static int CombineSigned32(ushort high, ushort low) =>
        unchecked((int)(((uint)high << 16) | low));

    private static string DescribeAlarm(ushort alarm)
    {
        if (alarm == 0)
        {
            return "Không có Alarm";
        }

        var descriptions = new List<string>();
        if ((alarm & 0x0001) != 0)
            descriptions.Add("Quá dòng hoặc không phát hiện motor; kiểm tra cặp cuộn A+/A-, B+/B- và khởi động lại driver");
        if ((alarm & 0x0002) != 0)
            descriptions.Add("Quá áp nguồn");
        if ((alarm & 0x0040) != 0)
            descriptions.Add("Lỗi mạch lấy mẫu dòng");
        if ((alarm & 0x0080) != 0)
            descriptions.Add("Không khóa được trục hoặc đứt dây motor");
        if ((alarm & 0x0200) != 0)
            descriptions.Add("Lỗi EEPROM");
        if ((alarm & 0x0100) != 0)
            descriptions.Add("Lỗi Auto-tuning");

        return descriptions.Count > 0
            ? string.Join("; ", descriptions)
            : $"Alarm chưa định nghĩa 0x{alarm:X4}";
    }

    private AxisRuntime RequireOnline(AxisAddress address)
    {
        var axis = _state.GetAxis(address);
        if (!axis.IsOnline)
        {
            throw new InvalidOperationException($"Driver {address.DisplayId} đang Offline.");
        }
        return axis;
    }

    private void ClosePortWithoutLock(int line)
    {
        if (!_ports.TryRemove(line, out var port))
        {
            return;
        }

        try
        {
            if (port.IsOpen)
            {
                port.Close();
            }
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[RS485] Line {line}: lỗi khi đóng cổng cũ — {ex.Message}");
        }
        finally
        {
            try
            {
                port.Dispose();
            }
            catch
            {
                // Không để object SerialPort lỗi ngăn cản quá trình reconnect.
            }
        }
    }

    private static void ValidateLine(int line)
    {
        if (line is < 1 or > 4)
        {
            throw new ArgumentOutOfRangeException(
                nameof(line),
                "Line phải từ 1 đến 4.");
        }
    }

    private void ThrowIfDisposed()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;

        lock (_autoSync)
        {
            _autoCts?.Cancel();
            _autoCts?.Dispose();
            _autoCts = null;
            _autoTask = null;
        }

        lock (_jogSync)
        {
            foreach (var cts in _jogCts.Values)
            {
                cts.Cancel();
                cts.Dispose();
            }
            _jogCts.Clear();
            _jogTasks.Clear();
        }

        foreach (var cts in _pollCts.Values)
        {
            cts.Cancel();
            cts.Dispose();
        }
        _pollCts.Clear();
        _pollTasks.Clear();

        foreach (var port in _ports.Values)
        {
            try
            {
                if (port.IsOpen)
                {
                    port.Close();
                }
            }
            finally
            {
                port.Dispose();
            }
        }
        _ports.Clear();

        lock (_configSync)
        {
            _axisModeSettings.Clear();
        }

        _modeSettingsFileLock.Dispose();

        foreach (var lineLock in _lineLocks.Values)
        {
            lineLock.Dispose();
        }
    }

}
