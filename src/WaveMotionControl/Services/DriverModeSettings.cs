using WaveMotionControl.Models;

namespace WaveMotionControl.Services;

/// <summary>
/// Profile dùng chung cho từng driver.
///
/// EM2RS chỉ có một thanh ghi dòng Peak 0x0191. Service sẽ đổi dòng
/// trước khi HOME, MANUAL hoặc AUTO chạy. Các thông số HOME được lưu
/// cả trong phần mềm và trong EEPROM driver.
/// </summary>
public sealed record DriverModeSettings(
    double HomeCurrentAmps,
    double ManualCurrentAmps,
    double AutoCurrentAmps,
    int PulsesPerRevolution)
{
    public int HomeFastSpeedRpm { get; init; } = 120;
    public int HomeSlowSpeedRpm { get; init; } = 12;
    public int HomeAccelerationMsPer1000Rpm { get; init; } = 500;
    public int HomeDecelerationMsPer1000Rpm { get; init; } = 500;

    public static DriverModeSettings Default { get; } =
        new(3.0, 3.0, 3.0, 10_000)
        {
            HomeFastSpeedRpm = 120,
            HomeSlowSpeedRpm = 12,
            HomeAccelerationMsPer1000Rpm = 500,
            HomeDecelerationMsPer1000Rpm = 500
        };
}

public enum DriverOperatingMode
{
    Home,
    Manual,
    Auto
}
