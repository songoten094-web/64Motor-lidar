using System.Text.Json;
using WaveMotionControl.Services;
using WaveMotionControl.State;
using WaveMotionControl.Models;
using System.IO;
namespace LivoxHmi.App;

public sealed record ZoneMotorMapItem(string LivoxZoneId, int ClusterId, int WaveZoneColumn);

public sealed class ZoneMotorMapConfig
{
    public double SpeedMultiplier { get; set; } = 2.0;
    public double FastDurationSeconds { get; set; } = 30.0;
    public List<ZoneMotorMapItem> Mappings { get; set; } = new();
}

/// <summary>
/// Thin integration layer only. It translates a LIDAR Zone rising-edge into the
/// existing WaveMotion RS485 service. It never writes COM/Modbus registers itself.
/// </summary>
public sealed class ZoneFastEffectRouter
{
    private readonly IRs485Service _service;
    private readonly ApplicationState _state;
    private readonly Dictionary<string, ZoneMotorMapItem> _map;
    private readonly double _speedMultiplier;
    private readonly TimeSpan _duration;

    private ZoneFastEffectRouter(
        IRs485Service service,
        ApplicationState state,
        ZoneMotorMapConfig config)
    {
        _service = service;
        _state = state;
        _speedMultiplier = Math.Clamp(config.SpeedMultiplier, 1.01, 5.0);
        _duration = TimeSpan.FromSeconds(Math.Clamp(config.FastDurationSeconds, 1.0, 600.0));
        _map = config.Mappings
            .Where(x => !string.IsNullOrWhiteSpace(x.LivoxZoneId) && x.ClusterId > 0 && x.WaveZoneColumn > 0)
            .GroupBy(x => x.LivoxZoneId, StringComparer.OrdinalIgnoreCase)
            .ToDictionary(g => g.Key, g => g.First(), StringComparer.OrdinalIgnoreCase);
    }

    public static ZoneFastEffectRouter Load(
        IRs485Service service,
        ApplicationState state,
        string path)
    {
        ZoneMotorMapConfig config;
        if (File.Exists(path))
        {
            var json = File.ReadAllText(path);
            config = JsonSerializer.Deserialize<ZoneMotorMapConfig>(json,
                new JsonSerializerOptions { PropertyNameCaseInsensitive = true })
                ?? new ZoneMotorMapConfig();
        }
        else
        {
            config = new ZoneMotorMapConfig();
        }

        return new ZoneFastEffectRouter(service, state, config);
    }

    public async Task OnZoneActiveAsync(string livoxZoneId, CancellationToken cancellationToken = default)
    {
        if (!_map.TryGetValue(livoxZoneId, out var route))
        {
            _state.WriteLog(LogLevel.Warning,
                $"[LIVOX LINK] {livoxZoneId}: chưa có mapping Zone → motor cluster.");
            return;
        }

        try
        {
            await _service.TriggerZoneFastAsync(
                route.ClusterId,
                route.WaveZoneColumn - 1,
                _speedMultiplier,
                _duration,
                cancellationToken).ConfigureAwait(false);
        }
        catch (Exception ex)
        {
            _state.WriteLog(LogLevel.Error,
                $"[LIVOX LINK] {livoxZoneId} → C{route.ClusterId}/Z{route.WaveZoneColumn}: {ex.Message}");
        }
    }
}
