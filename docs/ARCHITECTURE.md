# v3.4.3 Architecture

```text
MID-360
  -> Livox SDK2
  -> LivoxHmiBridge (C ABI)
  -> PointCloudFrame
  -> Zone spatial broad-phase
  -> static-map differential
  -> exact Surface/Zone volume test
  -> distance-adaptive point threshold
  -> 5-frame confirm
  -> one-shot Zone ACTIVE
  -> 5-frame clear
  -> re-arm
  -> HMI / output
```

## Realtime safety

- SDK callback only copies data into a bounded native frame buffer.
- Acquisition, detection and WPF rendering are separate loops.
- A stale detection frame can be replaced by a newer frame; the detector does not block LiDAR acquisition.
- The detector rejects points outside all Zone spatial cells before static-map ray matching and Surface projection.
- Zone spatial geometry is cached and rebuilt only when Surface/Zone geometry changes; signature checks are throttled.
- Per-Zone counters and accumulators use reusable arrays rather than per-frame dictionaries.
- Diagnostic string formatting is throttled to reduce GC pressure.
- Overview Zone rendering scales down automatically as Zone count grows; selected Zone remains full-detail.
- Holding inside an already ACTIVE Zone never retriggers it.

## Touch state

```text
FREE
  -> evidence for 5 consecutive frames
ACTIVE event (one shot)
  -> stay ACTIVE while touch remains
  -> clear for 5 consecutive frames
FREE / re-armed
```

## Persistence

Project JSON stores:
- MID-360 configuration path
- display transform / IMU orientation settings
- Surfaces
- Zones
- static-map settings
- touch detection settings

Surface/Zone/touch geometry remains in raw `MID360_SENSOR` coordinates.

## Native boundary

C# calls `LivoxHmiBridge.dll` through a small C ABI instead of directly binding Livox SDK2 callback types. This keeps native buffer ownership and callback lifetime isolated from WPF.
