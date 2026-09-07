using WaveMotionControl.Models;

namespace WaveMotionControl.Services;

/// <summary>
/// Chức năng test con trượt gần đều bằng bảng 16 PR tự Jump trong driver.
/// </summary>
public interface IUniformSliderMotionService
{
    UniformSliderMotionPlan PreviewUniformSliderMotion(
        AxisAddress axis,
        UniformSliderMotionSettings settings);

    Task SetUniformMechanicalZeroAsync(
        AxisAddress axis,
        CancellationToken cancellationToken = default);

    Task StartUniformSliderMotionAsync(
        AxisAddress axis,
        UniformSliderMotionSettings settings,
        CancellationToken cancellationToken = default);
}
