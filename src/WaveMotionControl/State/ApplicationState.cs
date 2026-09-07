using WaveMotionControl.Models;

namespace WaveMotionControl.State;

public sealed class ApplicationState
{
    private readonly object _lock = new();
    private readonly Dictionary<AxisAddress, AxisRuntime> _axes =
        AxisAddress.All().ToDictionary(a => a, a => new AxisRuntime(a));

    public ApplicationState()
    {
        Lines = Enumerable.Range(1, 4)
            .Select(i => new LineConnection
            {
                LineNumber = i,
                PortName = $"COM{i}",
                BaudRate = 115200
            })
            .ToArray();
    }

    public event EventHandler? StateChanged;
    public event Action<LogEntry>? LogAdded;

    public LineConnection[] Lines { get; }
    public IReadOnlyCollection<AxisRuntime> Axes
    {
        get
        {
            lock (_lock)
            {
                return _axes.Values.ToArray();
            }
        }
    }

    public AxisAddress SelectedAxis { get; set; } = new(1, 1);

    public AxisRuntime GetAxis(AxisAddress address)
    {
        lock (_lock)
        {
            return _axes[address];
        }
    }

    public IEnumerable<AxisRuntime> GetAxesForLine(int line)
    {
        lock (_lock)
        {
            return _axes.Values.Where(a => a.Address.Line == line).OrderBy(a => a.Address.SlaveId).ToArray();
        }
    }

    public void NotifyStateChanged() => StateChanged?.Invoke(this, EventArgs.Empty);

    public void WriteLog(LogLevel level, string message)
    {
        LogAdded?.Invoke(new LogEntry(DateTime.Now, level, message));
    }

    public int OnlineCount
    {
        get
        {
            lock (_lock)
            {
                return _axes.Values.Count(a => a.IsOnline);
            }
        }
    }

    public int HomedCount
    {
        get
        {
            lock (_lock)
            {
                return _axes.Values.Count(a => a.IsHomed);
            }
        }
    }

    public int HomingCount
    {
        get
        {
            lock (_lock)
            {
                return _axes.Values.Count(a => a.State == AxisMotionState.Homing);
            }
        }
    }

    public int AlarmCount
    {
        get
        {
            lock (_lock)
            {
                return _axes.Values.Count(a => a.State == AxisMotionState.Alarm);
            }
        }
    }
}
