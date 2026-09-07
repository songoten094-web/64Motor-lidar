namespace LivoxHmi.Core;

public interface ITouchDetector
{
    IReadOnlyList<TouchEvidence> Detect(PointCloudFrame frame, ProjectDefinition project, StaticMapSnapshot? staticMap, CancellationToken ct);
}

/// <summary>
/// STATIC TOUCH detector.
///
/// Goal: behave like a sensitive touch sensor without one-point false triggers.
/// The detector intentionally keeps only three realtime gates:
///   1) the live return must be closer than the locked static map on the same LiDAR ray;
///   2) the return must be inside an enabled Zone volume;
///   3) the current frame must meet a very small distance-adaptive point threshold.
///
/// Five-frame confirmation and five-frame clear/re-arm behavior are handled by ZoneEngine.
///
/// Performance strategy for many Zones:
///   - build a cached 3D uniform-grid spatial index from Zone sensor-space AABBs;
///   - reject points outside all Zone cells before static-map ray matching;
///   - project only dynamic points into the few Zones found in the same spatial cell;
///   - never tessellate or rebuild Zone geometry in the hot loop.
///
/// All geometry remains in MID360_SENSOR coordinates.
/// </summary>
public sealed class StaticTouchDetector : ITouchDetector
{
    private const double ZoneAabbPaddingMeters = 0.060;
    private const double SpatialCellMeters = 0.50;

    private const int AzimuthBins = 720;   // 0.5 degree
    private const int ElevationBins = 360; // 0.5 degree
    private const double DegToRad = Math.PI / 180.0;

    private StaticMapSnapshot? _indexedMap;
    private Dictionary<int, List<StaticRaySample>> _rayGrid = new();

    private int _zoneGeometrySignature;
    private readonly Dictionary<CellKey, List<ZoneRef>> _zoneGrid = new();
    private readonly Dictionary<string, ZoneRef> _zoneRefs = new(StringComparer.OrdinalIgnoreCase);
    private ZoneRef[] _zoneRefArray = Array.Empty<ZoneRef>();

    // Reused per-frame buffers. These eliminate Dictionary allocations proportional to Zone count.
    private int[] _counts = Array.Empty<int>();
    private PointAccumulator[] _sums = Array.Empty<PointAccumulator>();
    private double[] _bestDelta = Array.Empty<double>();
    private int _diagnosticFrameCounter;
    private int _geometryCheckFrameCounter;

    public bool Enabled { get; set; } = true;
    public string Diagnostics { get; private set; } = "STATIC TOUCH: idle";

    public IReadOnlyList<TouchEvidence> Detect(PointCloudFrame frame, ProjectDefinition project, StaticMapSnapshot? staticMap, CancellationToken ct)
    {
        if (!Enabled || staticMap is null || staticMap.Points.Count == 0)
            return Empty("STATIC TOUCH: OFF / NO STATIC MAP");

        bool hasEnabledZone = false;
        for (int i = 0; i < project.Zones.Count; i++)
        {
            var z = project.Zones[i];
            if (z.Enabled && z.Polygon.Count >= 3) { hasEnabledZone = true; break; }
        }
        if (!hasEnabledZone)
            return Empty("STATIC TOUCH: NO ENABLED ZONE");

        EnsureStaticRangeIndex(staticMap);
        if (_rayGrid.Count == 0)
            return Empty("STATIC TOUCH: EMPTY STATIC RANGE INDEX");

        // Geometry hashing scales with Zone count, so do it at ~1.5 Hz instead of every frame.
        // A newly edited Zone becomes active within well under one second without taxing realtime detection.
        if (_zoneGrid.Count == 0 || (++_geometryCheckFrameCounter % 12) == 0)
            EnsureZoneSpatialIndex(project);
        if (_zoneGrid.Count == 0)
            return Empty("STATIC TOUCH: NO INDEXABLE ZONE");

        EnsureScratchBuffers(_zoneRefArray.Length);
        Array.Clear(_counts);
        Array.Clear(_sums);
        Array.Clear(_bestDelta);

        int input = 0, nearZone = 0, rayMatched = 0, dynamic = 0, projected = 0;
        double zoneBackMeters = Math.Max(0.001, project.Detection.ZoneBackMm / 1000.0);
        double zoneFrontMeters = Math.Max(0.001, project.Detection.ZoneFrontMm / 1000.0);

        // Spatial lookup is very cheap, so preserve sensitivity by scanning virtually every live
        // return. Only exceptionally huge frames are thinned, and only before the cheap lookup.
        int stride = Math.Max(1, (int)Math.Ceiling(frame.Points.Count / 120000.0));

        for (int i = 0; i < frame.Points.Count; i += stride)
        {
            if ((i & 4095) == 0) ct.ThrowIfCancellationRequested();

            var p = frame.Points[i];
            if (!Finite(p) || !LivoxTagDecoder.Decode(p.Tag).IsNormal) continue;
            input++;

            double liveRange = Range(p);
            if (liveRange < 0.20 || liveRange > 30.0) continue;

            // Broad phase FIRST: points that cannot possibly touch any Zone never pay for
            // spherical static-map lookup or Surface projection.
            if (!_zoneGrid.TryGetValue(ToCell(p), out var candidateZones) || candidateZones.Count == 0)
                continue;
            nearZone++;

            if (!TryGetStaticRange(p, liveRange, out double staticRange, out double angularErrorDeg))
                continue;
            rayMatched++;

            double deltaRange = staticRange - liveRange;
            if (deltaRange < DynamicRangeThresholdMeters(liveRange, angularErrorDeg))
                continue;
            dynamic++;

            foreach (var zr in candidateZones)
            {
                // Exact AABB check removes duplicate/edge grid occupants before expensive projection.
                if (!zr.Bounds.Contains(p)) continue;

                try
                {
                    var pr = SurfaceGeometry.Project(p, zr.Surface);
                    if (pr.N < -zoneBackMeters || pr.N > zoneFrontMeters) continue;
                    if (!SurfaceGeometry.IsInside(zr.Zone, pr.U, pr.V)) continue;

                    projected++;
                    int zi = zr.Index;
                    _counts[zi]++;
                    _sums[zi] = _sums[zi].Add(p);
                    if (deltaRange > _bestDelta[zi]) _bestDelta[zi] = deltaRange;
                }
                catch (DllNotFoundException)
                {
                    // Optional OCCT unavailable: other Plane/Cylinder Zones continue normally.
                }
                catch (InvalidOperationException) when (zr.Surface.CurvedKind == CurvedSurfaceKind.OcctBSpline)
                {
                }
                catch
                {
                    // A malformed Surface must never stop the realtime detector.
                }
            }
        }

        int maxEvidence = Math.Max(1, project.Detection.MaxTouchEvidencePerFrame);
        var output = new List<TouchEvidence>(Math.Min(_zoneRefArray.Length, maxEvidence));
        bool updateDiagnostics = (++_diagnosticFrameCounter % 9) == 0; // ~2 Hz at 18 Hz detection
        List<string>? diagZones = updateDiagnostics ? new List<string>(6) : null;

        for (int zi = 0; zi < _zoneRefArray.Length; zi++)
        {
            var zr = _zoneRefArray[zi];
            int count = _counts[zi];
            double range = zr.CenterRange;
            int required = RequiredPointCount(range, project.Detection);
            bool evidence = count >= required;

            if (evidence && _sums[zi].Count > 0 && output.Count < maxEvidence)
            {
                var marker = _sums[zi].Centroid();
                double countScore = Math.Clamp((double)count / Math.Max(required * 2.0, 1.0), 0.0, 1.0);
                double deltaScore = Math.Clamp(_bestDelta[zi] / 0.10, 0.0, 1.0);
                double confidence = Math.Clamp(0.60 * countScore + 0.40 * deltaScore, 0.20, 0.99);
                output.Add(new TouchEvidence(marker, confidence, zr.Zone.Id));
            }

            if (updateDiagnostics && diagZones is not null && diagZones.Count < 6 && (count > 0 || evidence))
                diagZones.Add($"{zr.Zone.Id}:{count}/{required}@{range:0.0}m");
        }

        if (updateDiagnostics)
        {
            Diagnostics = $"STATIC TOUCH | zones {_zoneRefArray.Length} cells {_zoneGrid.Count} | in {input} nearZ {nearZone} ray {rayMatched} dyn {dynamic} vol {projected} | " +
                          (diagZones is null || diagZones.Count == 0 ? "no touch evidence" : string.Join(" ", diagZones));
        }

        return output;
    }

    private static int RequiredPointCount(double rangeMeters, DetectionSettings settings)
    {
        if (rangeMeters <= settings.NearRangeMeters) return Math.Max(2, settings.NearMinPoints);
        if (rangeMeters <= settings.MidRangeMeters) return Math.Max(2, settings.MidMinPoints);
        return Math.Max(2, settings.FarMinPoints);
    }

    private void EnsureZoneSpatialIndex(ProjectDefinition project)
    {
        int signature = ComputeGeometrySignature(project);
        if (signature == _zoneGeometrySignature && _zoneGrid.Count > 0) return;

        _zoneGeometrySignature = signature;
        _zoneGrid.Clear();
        _zoneRefs.Clear();

        // Built only when Zone/Surface geometry changes, never once per detection frame.
        var surfaces = new Dictionary<string, SurfaceDefinition>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < project.Surfaces.Count; i++)
        {
            var s = project.Surfaces[i];
            if (s.Frame == "MID360_SENSOR") surfaces[s.Id] = s;
        }

        int zoneIndex = 0;
        for (int zoneLoopIndex = 0; zoneLoopIndex < project.Zones.Count; zoneLoopIndex++)
        {
            var zone = project.Zones[zoneLoopIndex];
            if (!zone.Enabled || zone.Polygon.Count < 3) continue;
            if (!surfaces.TryGetValue(zone.SurfaceId, out var surface)) continue;

            try
            {
                var sample = SampleZoneSensorPoints(zone, surface);
                if (sample.Count == 0) continue;

                double pad = Math.Max(project.Detection.ZoneFrontMm, project.Detection.ZoneBackMm) / 1000.0 + ZoneAabbPaddingMeters;
                double minX = sample.Min(p => p.X) - pad;
                double minY = sample.Min(p => p.Y) - pad;
                double minZ = sample.Min(p => p.Z) - pad;
                double maxX = sample.Max(p => p.X) + pad;
                double maxY = sample.Max(p => p.Y) + pad;
                double maxZ = sample.Max(p => p.Z) + pad;

                var bounds = new Aabb(minX, minY, minZ, maxX, maxY, maxZ);
                var centerUv = new UvPoint(zone.Polygon.Average(q => q.U), zone.Polygon.Average(q => q.V));
                var center = SurfaceGeometry.FromUv(surface, centerUv.U, centerUv.V);
                var zr = new ZoneRef(zoneIndex++, zone, surface, bounds, Range(center));
                _zoneRefs[zone.Id] = zr;

                int x0 = CellCoord(minX), x1 = CellCoord(maxX);
                int y0 = CellCoord(minY), y1 = CellCoord(maxY);
                int z0 = CellCoord(minZ), z1 = CellCoord(maxZ);

                for (int x = x0; x <= x1; x++)
                for (int y = y0; y <= y1; y++)
                for (int z = z0; z <= z1; z++)
                {
                    var key = new CellKey(x, y, z);
                    if (!_zoneGrid.TryGetValue(key, out var list))
                        _zoneGrid[key] = list = new List<ZoneRef>(2);
                    list.Add(zr);
                }
            }
            catch
            {
                // Skip only the Zone that cannot be represented in sensor coordinates.
            }
        }

        _zoneRefArray = _zoneRefs.Values.OrderBy(z => z.Index).ToArray();
        EnsureScratchBuffers(_zoneRefArray.Length);
    }

    private void EnsureScratchBuffers(int count)
    {
        if (_counts.Length == count) return;
        _counts = new int[count];
        _sums = new PointAccumulator[count];
        _bestDelta = new double[count];
    }

    private static List<Point3D> SampleZoneSensorPoints(ZoneDefinition zone, SurfaceDefinition surface)
    {
        var points = new List<Point3D>(zone.Polygon.Count * 5 + 1);
        const int edgeSegments = 4;

        for (int i = 0; i < zone.Polygon.Count; i++)
        {
            var a = zone.Polygon[i];
            var b = zone.Polygon[(i + 1) % zone.Polygon.Count];
            for (int s = 0; s < edgeSegments; s++)
            {
                double t = s / (double)edgeSegments;
                double u = a.U + (b.U - a.U) * t;
                double v = a.V + (b.V - a.V) * t;
                points.Add(SurfaceGeometry.FromUv(surface, u, v));
            }
        }

        var center = new UvPoint(zone.Polygon.Average(q => q.U), zone.Polygon.Average(q => q.V));
        points.Add(SurfaceGeometry.FromUv(surface, center.U, center.V));
        return points;
    }

    private static int ComputeGeometrySignature(ProjectDefinition project)
    {
        var settings = project.Detection;
        var h = new HashCode();
        h.Add(project.Zones.Count);
        h.Add(project.Surfaces.Count);
        h.Add(settings.ZoneFrontMm);
        h.Add(settings.ZoneBackMm);

        for (int i = 0; i < project.Zones.Count; i++)
        {
            var z = project.Zones[i];
            h.Add(z.Id, StringComparer.OrdinalIgnoreCase);
            h.Add(z.SurfaceId, StringComparer.OrdinalIgnoreCase);
            h.Add(z.Enabled);
            h.Add(z.Polygon.Count);
            for (int q = 0; q < z.Polygon.Count; q++)
            {
                h.Add(z.Polygon[q].U);
                h.Add(z.Polygon[q].V);
            }
        }

        for (int i = 0; i < project.Surfaces.Count; i++)
        {
            var surf = project.Surfaces[i];
            h.Add(surf.Id, StringComparer.OrdinalIgnoreCase);
            h.Add(surf.Frame, StringComparer.Ordinal);
            h.Add(surf.Type);
            h.Add(surf.CurvedKind);
            h.Add(surf.Origin.X); h.Add(surf.Origin.Y); h.Add(surf.Origin.Z);
            h.Add(surf.Normal.X); h.Add(surf.Normal.Y); h.Add(surf.Normal.Z);
            h.Add(surf.UAxis.X); h.Add(surf.UAxis.Y); h.Add(surf.UAxis.Z);
            h.Add(surf.VAxis.X); h.Add(surf.VAxis.Y); h.Add(surf.VAxis.Z);
            h.Add(surf.WidthMeters); h.Add(surf.HeightMeters);
            h.Add(surf.CurvatureRadiusMeters); h.Add(surf.CurvatureSign);
            h.Add(surf.OcctBulgeMeters); h.Add(surf.OcctTwistMeters);
            h.Add(surf.OcctFitControlPoints.Count);
        }
        return h.ToHashCode();
    }

    private void EnsureStaticRangeIndex(StaticMapSnapshot map)
    {
        if (ReferenceEquals(map, _indexedMap)) return;

        _indexedMap = map;
        _rayGrid = new Dictionary<int, List<StaticRaySample>>();

        foreach (var p in map.Points)
        {
            if (!Finite(p)) continue;
            double range = Range(p);
            if (range < 0.20 || range > 30.0) continue;

            var dir = UnitDirection(p, range);
            int key = RayKey(AzimuthBin(dir.X, dir.Y), ElevationBin(dir.Z));
            if (!_rayGrid.TryGetValue(key, out var bucket))
                _rayGrid[key] = bucket = new List<StaticRaySample>();
            bucket.Add(new StaticRaySample(dir.X, dir.Y, dir.Z, range));
        }

        foreach (var key in _rayGrid.Keys.ToArray())
        {
            var bucket = _rayGrid[key];
            bucket.Sort((a, b) => a.Range.CompareTo(b.Range));
            if (bucket.Count > 10)
                _rayGrid[key] = bucket.Take(10).ToList();
        }
    }

    private bool TryGetStaticRange(Point3D p, double liveRange, out double staticRange, out double angularErrorDeg)
    {
        var live = UnitDirection(p, liveRange);
        int az = AzimuthBin(live.X, live.Y);
        int el = ElevationBin(live.Z);

        int radiusBins = liveRange >= 5.0 ? 3 : 2;
        double maxAngleDeg = liveRange >= 5.0 ? 1.35 : 1.05;
        double minDot = Math.Cos(maxAngleDeg * DegToRad);

        bool found = false;
        double bestDot = -1.0;
        double bestRange = 0.0;

        for (int de = -radiusBins; de <= radiusBins; de++)
        {
            int ee = el + de;
            if (ee < 0 || ee >= ElevationBins) continue;

            for (int da = -radiusBins; da <= radiusBins; da++)
            {
                int aa = WrapAzimuthBin(az + da);
                if (!_rayGrid.TryGetValue(RayKey(aa, ee), out var bucket)) continue;

                foreach (var s in bucket)
                {
                    double dot = live.X * s.Dx + live.Y * s.Dy + live.Z * s.Dz;
                    if (dot < minDot || dot <= bestDot) continue;
                    bestDot = dot;
                    bestRange = s.Range;
                    found = true;
                }
            }
        }

        if (!found)
        {
            staticRange = 0;
            angularErrorDeg = double.PositiveInfinity;
            return false;
        }

        staticRange = bestRange;
        angularErrorDeg = Math.Acos(Math.Clamp(bestDot, -1.0, 1.0)) / DegToRad;
        return true;
    }

    // Sensitivity-first: Zone geometry is already a strong gate, so static-map delta threshold stays low.
    private static double DynamicRangeThresholdMeters(double rangeMeters, double angularErrorDeg)
    {
        double rangeThreshold = rangeMeters <= 2.0
            ? 0.022
            : rangeMeters >= 8.0
                ? 0.040
                : 0.022 + (rangeMeters - 2.0) * (0.018 / 6.0);
        double angularPad = Math.Clamp(angularErrorDeg / 1.35, 0.0, 1.0) * 0.006;
        return rangeThreshold + angularPad;
    }

    private IReadOnlyList<TouchEvidence> Empty(string s)
    {
        Diagnostics = s;
        return Array.Empty<TouchEvidence>();
    }

    private static int CellCoord(double v) => (int)Math.Floor(v / SpatialCellMeters);
    private static CellKey ToCell(Point3D p) => new(CellCoord(p.X), CellCoord(p.Y), CellCoord(p.Z));
    private static double Range(Point3D p) => Math.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
    private static bool Finite(Point3D p) => float.IsFinite(p.X) && float.IsFinite(p.Y) && float.IsFinite(p.Z);

    private static Direction3 UnitDirection(Point3D p, double range)
    {
        double inv = 1.0 / Math.Max(1e-9, range);
        return new Direction3(p.X * inv, p.Y * inv, p.Z * inv);
    }

    private static int AzimuthBin(double x, double y)
    {
        double az = Math.Atan2(y, x);
        if (az < 0) az += 2.0 * Math.PI;
        int bin = (int)Math.Floor(az / (2.0 * Math.PI) * AzimuthBins);
        return Math.Clamp(bin, 0, AzimuthBins - 1);
    }

    private static int ElevationBin(double zUnit)
    {
        double el = Math.Asin(Math.Clamp(zUnit, -1.0, 1.0));
        int bin = (int)Math.Floor((el + Math.PI / 2.0) / Math.PI * ElevationBins);
        return Math.Clamp(bin, 0, ElevationBins - 1);
    }

    private static int WrapAzimuthBin(int bin)
    {
        bin %= AzimuthBins;
        if (bin < 0) bin += AzimuthBins;
        return bin;
    }

    private static int RayKey(int azimuthBin, int elevationBin) => elevationBin * AzimuthBins + azimuthBin;

    private readonly record struct Direction3(double X, double Y, double Z);
    private readonly record struct StaticRaySample(double Dx, double Dy, double Dz, double Range);
    private readonly record struct CellKey(int X, int Y, int Z);
    private readonly record struct ZoneRef(int Index, ZoneDefinition Zone, SurfaceDefinition Surface, Aabb Bounds, double CenterRange);

    private readonly record struct Aabb(double MinX, double MinY, double MinZ, double MaxX, double MaxY, double MaxZ)
    {
        public bool Contains(Point3D p) =>
            p.X >= MinX && p.X <= MaxX &&
            p.Y >= MinY && p.Y <= MaxY &&
            p.Z >= MinZ && p.Z <= MaxZ;
    }

    private readonly record struct PointAccumulator(double X, double Y, double Z, int Count)
    {
        public PointAccumulator Add(Point3D p) => new(X + p.X, Y + p.Y, Z + p.Z, Count + 1);
        public Point3D Centroid() => Count <= 0
            ? new Point3D(0, 0, 0, 0, 0)
            : new Point3D((float)(X / Count), (float)(Y / Count), (float)(Z / Count), 0, 0);
    }
}
