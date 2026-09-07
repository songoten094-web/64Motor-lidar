using WaveMotionControl.Models;

namespace WaveMotionControl.Services;

/// <summary>
/// Giao diện bổ sung cho cấu hình dòng theo chế độ và Pulse/vòng dùng chung.
/// Tách khỏi IRs485Service để không làm hỏng DemoRs485Service hoặc các trang cũ.
/// </summary>
public interface IModeDriverSettingsService
{
    Task SaveCompleteDriverSettingsAsync(
        IEnumerable<AxisAddress> axes,
        int diPinIndex,
        bool activeLowNC,
        int standbyPercent,
        double autoSpeedRps,
        double autoAccRps2,
        DriverModeSettings settings,
        CancellationToken cancellationToken = default);

    Task SaveModeDriverSettingsAsync(
        IEnumerable<AxisAddress> axes,
        DriverModeSettings settings,
        CancellationToken cancellationToken = default);

    Task<DriverModeSettings> ReadModeDriverSettingsAsync(
        AxisAddress address,
        CancellationToken cancellationToken = default);

    int GetConfiguredPulsesPerRevolution(AxisAddress address);
}
