# v3.4.5 — One-frame ACTIVE / far-range trigger fix

Root cause found in the uploaded source:

1. `TouchEvidenceModel` draws a red point as soon as `StaticTouchDetector` returns evidence for a single frame.
2. `ZoneEngine` previously required `ConfirmFrames = 3` consecutive evidence frames before producing `ZoneState.Active` and routing the event to WaveMotion.
3. At long range the hand return is sparse/intermittent, so red evidence could be visible without ever accumulating 3 consecutive frames. A full body produces denser consecutive evidence, therefore WaveMotion triggered.
4. The detection loop was also silently clamping `DetectionFps=30` to 20 Hz.

Changes:

- `DetectionFps = 30`
- `ConfirmFrames = 1`
- `ReleaseFrames = 8`
- Detection loop maximum raised from 20 Hz to 30 Hz.
- Existing adaptive raw-point threshold is kept: near=4, mid=3, far=2. Therefore one raw LiDAR point still cannot trigger.
- WaveMotion RS485/Modbus/16PR structure is unchanged.

New behavior:

`red TouchEvidence point -> Zone ACTIVE in the same detection cycle -> rising-edge event -> ZoneFastEffectRouter -> TriggerZoneFastAsync`.

The 8-frame release latch prevents brief far-range dropouts from immediately re-arming the Zone.
