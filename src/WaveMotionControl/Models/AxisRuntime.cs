namespace WaveMotionControl.Models;

public enum AxisMotionState
{
    Offline,
    Online,
    Homing,
    Homed,
    JoggingForward,
    JoggingReverse,
    Moving,
    Alarm
}

public sealed class AxisRuntime
{
    public AxisRuntime(AxisAddress address)
    {
        Address = address;
    }

    public AxisAddress Address { get; }
    public AxisMotionState State { get; set; } = AxisMotionState.Offline;
    public double PositionRevolutions { get; set; }
    public int VelocityRpm { get; set; }
    public string LastCommand { get; set; } = "—";
    public string AlarmText { get; set; } = string.Empty;

    public bool IsOnline => State != AxisMotionState.Offline;
    public bool IsHomed => State == AxisMotionState.Homed;
}
