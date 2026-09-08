
namespace LivoxHmi.Core;

public readonly record struct Point3D(float X, float Y, float Z, byte Reflectivity, byte Tag);

public sealed record PointCloudFrame(
    DateTimeOffset Timestamp,
    uint LidarHandle,
    IReadOnlyList<Point3D> Points,
    int DroppedPoints,
    int PacketCount);

public sealed class SurfaceDefinition
{
    // Geometry is ALWAYS stored in the native MID-360 sensor frame.
    // Viewer/IMU alignment never modifies persisted surface coordinates.
    public string Frame { get; set; } = "MID360_SENSOR";
    public string Id { get; set; } = "S01";
    public string Name { get; set; } = "Surface 01";
    public SurfaceType Type { get; set; } = SurfaceType.Plane;
    public Point3D Origin { get; set; }
    public Point3D Normal { get; set; } = new(0, 0, 1, 0, 0);
    public Point3D UAxis { get; set; } = new(1, 0, 0, 0, 0);
    public Point3D VAxis { get; set; } = new(0, 1, 0, 0, 0);
    public double ValidationRmseMm { get; set; }
    public double WidthMeters { get; set; } = 2.0;
    public double HeightMeters { get; set; } = 1.5;
    // Curved surface foundation. WidthMeters remains physical arc length in U, HeightMeters remains V length.
    public CurvedSurfaceKind CurvedKind { get; set; } = CurvedSurfaceKind.Cylinder;
    public double CurvatureRadiusMeters { get; set; } = 1.0;
    // +1 bends toward +Normal center side, -1 bends the opposite way. Stored in SENSOR geometry.
    public double CurvatureSign { get; set; } = 1.0;
    // OCCT B-Spline freeform parameters. Geometry is generated directly in MID360_SENSOR.
    public double OcctBulgeMeters { get; set; } = 0.20;
    public double OcctTwistMeters { get; set; } = 0.00;
    // Optional fitted OCCT grid, persisted in raw MID360_SENSOR XYZ. When present,
    // it replaces the manual Bulge/Twist seed for the actual B-Spline geometry.
    public int OcctFitUCount { get; set; }
    public int OcctFitVCount { get; set; }
    public List<Point3D> OcctFitControlPoints { get; set; } = new();
    public int OcctFitSourcePointCount { get; set; }
    public double OcctFitRmseMm { get; set; }
}

public enum SurfaceType { Plane, Curved }
public enum CurvedSurfaceKind { Cylinder, OcctBSpline }

public sealed class ZoneDefinition
{
    public string Id { get; set; } = "Z01";
    public string Name { get; set; } = "Zone 01";
    public string SurfaceId { get; set; } = "S01";
    public ZoneType Type { get; set; } = ZoneType.Rectangle;
    public List<UvPoint> Polygon { get; set; } = new();
    public bool Enabled { get; set; } = true;
    // Legacy V2.2 per-Zone extrusion values retained only for project-file backward compatibility.
    // V2.4 runtime ignores these values and uses DetectionSettings.SurfaceVolumeFrontMm/BackMm globally.
    public double ExtrudeFrontMeters { get; set; } = 0.45;
    public double ExtrudeBackMeters { get; set; } = 0.10;
}

public enum ZoneType { Rectangle, Polygon }

public readonly record struct UvPoint(double U, double V);

public sealed class ProjectDefinition
{
    public int SchemaVersion { get; set; } = 14;
    public string Id { get; set; } = "MACHINE_A";
    public string Name { get; set; } = "Machine A";
    public string Mid360ConfigPath { get; set; } = "config/mid360_config.json";
    public CalibrationDefinition Calibration { get; set; } = new();
    public List<SurfaceDefinition> Surfaces { get; set; } = new();
    public List<ZoneDefinition> Zones { get; set; } = new();
    public DetectionSettings Detection { get; set; } = new();
    public StaticMapDefinition StaticMap { get; set; } = new();
    public StaticMapSettings StaticMapSettings { get; set; } = new();
}

public sealed class CalibrationDefinition
{
    // DISPLAY ONLY: Sensor -> Viewer rotation matrix. Origin is ALWAYS the MID-360 origin.
    // Geometry/detection stays in raw MID-360 Sensor XYZ; this transform is never persisted into Surface/Zone coordinates.
    public double[] LidarToWorld { get; set; } =
        { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

    public bool ImuCalibrated { get; set; }
    public bool CoordinateFrameLocked { get; set; }
    public bool ZPositiveDown { get; set; } = true;
    public double UserXRotationDeg { get; set; }
    public double ImuRollDeg { get; set; }
    public double ImuPitchDeg { get; set; }
    public double ImuTiltDeg { get; set; }
}

public sealed class DetectionSettings
{
    // Realtime loop. 18 Hz + 5-frame confirmation gives ~278 ms nominal touch latency.
    public int DetectionFps { get; set; } = 18;
    public int ConfirmFrames { get; set; } = 5;
    public int ReleaseFrames { get; set; } = 5;

    // Symmetric-ish interaction volume around each Zone surface.
    public double ZoneFrontMm { get; set; } = 125;
    public double ZoneBackMm { get; set; } = 125;

    // Sensitivity-first distance-adaptive evidence threshold. Never allow one-point trigger.
    public double NearRangeMeters { get; set; } = 3.0;
    public double MidRangeMeters { get; set; } = 6.0;
    public int NearMinPoints { get; set; } = 4;
    public int MidMinPoints { get; set; } = 3;
    public int FarMinPoints { get; set; } = 2;

    public int MaxTouchEvidencePerFrame { get; set; } = 128;
}

/// <summary>
/// Per-frame evidence that a specific Zone contains enough dynamic points to count as a touch candidate.
/// ZoneEngine performs the 5-frame confirmation and 5-frame clear/re-arm state transition.
/// </summary>
public readonly record struct TouchEvidence(Point3D Position, double Confidence, string ZoneId);

public enum ZoneState { Free, Active }

public sealed record ZoneEvent(
    string ZoneId,
    ZoneState State,
    DateTimeOffset Timestamp);

public sealed class ZoneRuntimeState
{
    public ZoneState State { get; set; } = ZoneState.Free;
    public int PresentFrames { get; set; }
    public int AbsentFrames { get; set; }
}
