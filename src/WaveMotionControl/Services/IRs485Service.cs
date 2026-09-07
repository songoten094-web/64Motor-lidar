using WaveMotionControl.Models;

namespace WaveMotionControl.Services;

public enum JogDirection
{
    Forward,
    Reverse
}

public interface IRs485Service
{
    Task ConnectLineAsync(int line, string portName, int baudRate, CancellationToken cancellationToken = default);
    Task DisconnectLineAsync(int line, CancellationToken cancellationToken = default);
    Task HomeAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default);
    Task SetCurrentPositionAsOriginAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default);
    Task StartJogAsync(AxisAddress axis, JogDirection direction, int speedRpm, int acceleration, int deceleration, CancellationToken cancellationToken = default);
    Task StopAxisAsync(AxisAddress axis, CancellationToken cancellationToken = default);
    Task MoveRelativeRevolutionsAsync(AxisAddress axis, double signedRevolutions, int speedRpm, int pulsesPerRevolution, CancellationToken cancellationToken = default);
    Task StartAutoAsync(AutoProgram program, CancellationToken cancellationToken = default);
    Task SetLidarZoneAsync(int clusterId, int? zeroBasedZoneColumn, CancellationToken cancellationToken = default);
    Task TriggerZoneFastAsync(int clusterId, int zeroBasedZoneColumn, double speedMultiplier = 2.0, TimeSpan? duration = null, CancellationToken cancellationToken = default);
    Task PauseAutoAsync(bool paused, CancellationToken cancellationToken = default);
    Task StopAllAsync(bool quickStop, CancellationToken cancellationToken = default);

    Task SaveDriverConfigAsync(IEnumerable<AxisAddress> axes, int diPinIndex, bool activeLowNC, double peakCurrentAmps, int standbyPercent, double homingSpeedRps, double autoSpeedRps, double autoAccRps2, CancellationToken cancellationToken = default);
    Task ClearDriverConfigAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default);
    Task<(int diPinIndex, bool activeLowNC, double peakCurrentAmps, int standbyPercent, double homingSpeedRps, double autoSpeedRps, double autoAccRps2)> ReadDriverConfigAsync(AxisAddress axis, CancellationToken cancellationToken = default);
    Task ResetAlarmAsync(IEnumerable<AxisAddress> axes, CancellationToken cancellationToken = default);
}
