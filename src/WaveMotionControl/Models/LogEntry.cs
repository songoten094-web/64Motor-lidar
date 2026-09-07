namespace WaveMotionControl.Models;

public enum LogLevel
{
    Info,
    Ok,
    Warning,
    Error
}

public sealed record LogEntry(DateTime Timestamp, LogLevel Level, string Message);
