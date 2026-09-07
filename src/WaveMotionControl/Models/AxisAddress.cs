namespace WaveMotionControl.Models;

public readonly record struct AxisAddress(int Line, int SlaveId)
{
    public string DisplayId => $"{Line}.{SlaveId}";
    public int LinearIndex => (Line - 1) * 16 + (SlaveId - 1);

    public static IEnumerable<AxisAddress> All()
    {
        for (var line = 1; line <= 4; line++)
        {
            for (var slave = 1; slave <= 16; slave++)
            {
                yield return new AxisAddress(line, slave);
            }
        }
    }

    public static bool TryParse(string? text, out AxisAddress address)
    {
        address = default;
        if (string.IsNullOrWhiteSpace(text)) return false;

        var parts = text.Trim().Split('.', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != 2 ||
            !int.TryParse(parts[0], out var line) ||
            !int.TryParse(parts[1], out var slave))
        {
            return false;
        }

        if (line is < 1 or > 4 || slave is < 1 or > 16) return false;

        address = new AxisAddress(line, slave);
        return true;
    }

    public override string ToString() => DisplayId;
}
