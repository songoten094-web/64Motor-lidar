using LivoxHmi.Core;
using WaveMotionControl.Models;
using WaveMotionControl.Services;
using WaveMotionControl.State;

namespace LivoxHmi.App;

/// <summary>
/// Liên kết trực tiếp:
///
/// Surface S01 -> Cluster 1
/// Surface S02 -> Cluster 2
///
/// Zone.LocalIndex -> cột motor trong Cluster.
///
/// Không viết trực tiếp Modbus.
/// Toàn bộ truyền thông vẫn đi qua IRs485Service.
/// </summary>
public sealed class ZoneFastEffectRouter
{
    private readonly IRs485Service _service;
    private readonly ApplicationState _state;
    private readonly Func<ProjectDefinition> _projectProvider;

    private readonly double _speedMultiplier;
    private readonly TimeSpan _duration;

    public ZoneFastEffectRouter(
        IRs485Service service,
        ApplicationState state,
        Func<ProjectDefinition> projectProvider,
        double speedMultiplier = 2.0,
        double fastDurationSeconds = 30.0)
    {
        _service = service;
        _state = state;
        _projectProvider = projectProvider;

        _speedMultiplier =
            Math.Clamp(speedMultiplier, 1.01, 5.0);

        _duration = TimeSpan.FromSeconds(
            Math.Clamp(fastDurationSeconds, 1.0, 600.0));
    }

    public async Task OnZoneActiveAsync(
        string zoneId,
        CancellationToken cancellationToken = default)
    {
        var project = _projectProvider();

        var zone = project.Zones.FirstOrDefault(
            z => string.Equals(
                z.Id,
                zoneId,
                StringComparison.OrdinalIgnoreCase));

        if (zone is null)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[LIVOX LINK] Không tìm thấy Zone {zoneId}.");

            return;
        }

        var clusterId = SurfaceIdToClusterId(zone.SurfaceId);

        if (clusterId <= 0)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[LIVOX LINK] Surface {zone.SurfaceId} không có Cluster tương ứng.");

            return;
        }

        if (zone.LocalIndex <= 0)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[LIVOX LINK] {zone.Id}: LocalIndex không hợp lệ.");

            return;
        }

        // Service hiện tại dùng zero-based column.
        var zeroBasedColumn = zone.LocalIndex - 1;

        try
        {
            _state.WriteLog(
            LogLevel.Info,
            $"[LIVOX LINK] {zone.SurfaceId}/{zone.Id} " +
            $"-> Cluster {clusterId} / Column {zone.LocalIndex}");

            await _service.TriggerZoneFastAsync(
            clusterId,
            zeroBasedColumn,
            _speedMultiplier,
            _duration,
            cancellationToken);
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Error,
                $"[LIVOX LINK] {zone.SurfaceId}/{zone.Id} " +
                $"→ C{clusterId}/Column{zone.LocalIndex}: {ex.Message}");
        }
    }

    private static int SurfaceIdToClusterId(string? surfaceId)
    {
        if (string.IsNullOrWhiteSpace(surfaceId))
            return 0;

        // S01 -> 1
        // S02 -> 2
        // Surface03 cũng có thể xử lý nếu cuối chuỗi là số.
        var digits = new string(
            surfaceId
                .Where(char.IsDigit)
                .ToArray());

        if (!int.TryParse(digits, out var id))
            return 0;

        return id;
    }
}