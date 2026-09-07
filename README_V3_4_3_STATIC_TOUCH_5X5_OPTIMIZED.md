# LivoxHmi v3.4.3 — Static Touch 5×5 Optimized Clean

## Runtime

- Detection: static-map differential inside spatially indexed Zone volumes.
- Confirm: 5 consecutive evidence frames.
- Clear/re-arm: 5 consecutive clear frames.
- Holding inside a Zone never retriggers.

## Performance changes

- Per-Zone point counters/accumulators are reusable arrays, not per-frame dictionaries.
- ZoneEngine reuses its presence HashSet and no longer builds enabled-zone arrays/sets per frame.
- Zone geometry hash/index check runs about every 12 detection frames instead of every frame.
- Front/back volume distances are calculated once per frame, not for every candidate point.
- Diagnostics formatting is throttled to about 2 Hz.
- When Zone count grows, overview rendering automatically removes transparent faces, switches to one-side outlines, and coarsens curved display tessellation. Selected Zone keeps full detail.

These display reductions do not change touch geometry accuracy.

## Default sensitivity

- 0–3 m: 4 dynamic points
- 3–6 m: 3 dynamic points
- >6 m: 2 dynamic points
- ConfirmFrames = 5
- ReleaseFrames = 5
- DetectionFps = 18
