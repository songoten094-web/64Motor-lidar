namespace LivoxHmi.Core;

/// <summary>
/// One-shot touch state machine.
/// A Zone triggers once after ConfirmFrames consecutive evidence frames, then remains ACTIVE
/// until ReleaseFrames consecutive clear frames. Holding a hand/object in the Zone never retriggers.
///
/// Hot-path note: the scratch HashSet is reused to avoid per-frame set allocations when many
/// Zones are configured.
/// </summary>
public sealed class ZoneEngine
{
    private readonly Dictionary<string, ZoneRuntimeState> _runtime = new(StringComparer.OrdinalIgnoreCase);
    private readonly HashSet<string> _presentScratch = new(StringComparer.OrdinalIgnoreCase);
    private readonly object _gate = new();

    public IReadOnlyList<ZoneEvent> Update(
        IReadOnlyList<TouchEvidence> evidence,
        ProjectDefinition project,
        DateTimeOffset timestamp)
    {
        var events = new List<ZoneEvent>(2);

        lock (_gate)
        {
            _presentScratch.Clear();
            for (int i = 0; i < evidence.Count; i++)
            {
                var item = evidence[i];
                if (item.Confidence > 0 && !string.IsNullOrWhiteSpace(item.ZoneId))
                    _presentScratch.Add(item.ZoneId);
            }

            for (int i = 0; i < project.Zones.Count; i++)
            {
                var zone = project.Zones[i];
                if (!zone.Enabled) continue;

                if (!_runtime.TryGetValue(zone.Id, out var state))
                    _runtime[zone.Id] = state = new ZoneRuntimeState();

                if (_presentScratch.Contains(zone.Id))
                {
                    state.AbsentFrames = 0;
                    if (state.State == ZoneState.Free)
                    {
                        state.PresentFrames++;
                        if (state.PresentFrames >= project.Detection.ConfirmFrames)
                        {
                            state.PresentFrames = project.Detection.ConfirmFrames;
                            state.State = ZoneState.Active;
                            events.Add(new ZoneEvent(zone.Id, ZoneState.Active, timestamp));
                        }
                    }
                    else
                    {
                        // Already active: holding inside must never build another trigger count.
                        state.PresentFrames = project.Detection.ConfirmFrames;
                    }
                }
                else
                {
                    state.PresentFrames = 0;
                    if (state.State == ZoneState.Active)
                    {
                        state.AbsentFrames++;
                        if (state.AbsentFrames >= project.Detection.ReleaseFrames)
                        {
                            state.AbsentFrames = project.Detection.ReleaseFrames;
                            state.State = ZoneState.Free;
                            events.Add(new ZoneEvent(zone.Id, ZoneState.Free, timestamp));
                        }
                    }
                    else
                    {
                        state.AbsentFrames = 0;
                    }
                }
            }
        }

        return events;
    }

    public IReadOnlyDictionary<string, ZoneState> Snapshot()
    {
        lock (_gate)
            return _runtime.ToDictionary(x => x.Key, x => x.Value.State, StringComparer.OrdinalIgnoreCase);
    }

    public void Reset()
    {
        lock (_gate)
        {
            _runtime.Clear();
            _presentScratch.Clear();
        }
    }
}
