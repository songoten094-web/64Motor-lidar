using System.Numerics;

namespace LivoxHmi.Core;

/// <summary>
/// Single source of truth for the physical coordinate frame.
/// The MID-360 sensor origin is permanently kept at Project (0,0,0).
/// Only an orthonormal rotation is allowed: no translation and no scale.
///
/// Transform updates are atomic at frame level: the acquisition thread always
/// sees one immutable TransformState for an entire point-cloud conversion.
/// </summary>
public sealed class CoordinateEngine
{
    private sealed record TransformState(
        Matrix4x4 Forward,
        Matrix4x4 Inverse,
        Vector3 XAxisSensor,
        Vector3 YAxisSensor,
        Vector3 ZAxisSensor);

    private TransformState _state = CreateState(Matrix4x4.Identity);

    public Matrix4x4 LidarToWorld => Volatile.Read(ref _state).Forward;
    public Matrix4x4 WorldToLidar => Volatile.Read(ref _state).Inverse;
    public Vector3 ProjectXAxisInSensor => Volatile.Read(ref _state).XAxisSensor;
    public Vector3 ProjectYAxisInSensor => Volatile.Read(ref _state).YAxisSensor;
    public Vector3 ProjectZAxisInSensor => Volatile.Read(ref _state).ZAxisSensor;

    public void Configure(CalibrationDefinition calibration)
    {
        if (calibration?.LidarToWorld is not { Length: 16 } m)
            throw new ArgumentException("LidarToWorld must contain exactly 16 values.", nameof(calibration));

        var matrix = new Matrix4x4(
            (float)m[0], (float)m[1], (float)m[2], (float)m[3],
            (float)m[4], (float)m[5], (float)m[6], (float)m[7],
            (float)m[8], (float)m[9], (float)m[10], (float)m[11],
            (float)m[12], (float)m[13], (float)m[14], (float)m[15]);
        SetTransform(matrix, enforceRotationOnly: true);
    }

    /// <summary>
    /// Builds a right-handed Project frame from the IMU-derived Z direction.
    /// Project +X starts from physical MID-360 +X, projected onto the plane
    /// perpendicular to Project +Z, then receives the user's yaw offset.
    /// Project +Y is generated automatically as Z x X.
    /// </summary>
    public ProjectFrameResult ConfigureGravityFrame(Vector3 projectZInSensor, double userXRotationDeg)
    {
        if (!IsFinite(projectZInSensor) || projectZInSensor.LengthSquared() < 1e-8f)
            throw new ArgumentException("Project Z direction is invalid.", nameof(projectZInSensor));

        var z = Vector3.Normalize(projectZInSensor);
        var seed = Vector3.UnitX;
        var x0 = seed - Vector3.Dot(seed, z) * z;
        if (x0.LengthSquared() < 1e-6f)
        {
            seed = Vector3.UnitY;
            x0 = seed - Vector3.Dot(seed, z) * z;
        }
        x0 = Vector3.Normalize(x0);

        var yaw = (float)(Math.PI / 180.0 * userXRotationDeg);
        var x = RotateAroundAxis(x0, z, yaw);
        x -= Vector3.Dot(x, z) * z;
        x = Vector3.Normalize(x);

        var y = Vector3.Normalize(Vector3.Cross(z, x));
        x = Vector3.Normalize(Vector3.Cross(y, z));

        var matrix = CreateSensorToProjectRotation(x, y, z);
        SetTransform(matrix, enforceRotationOnly: true);
        return new ProjectFrameResult(x, y, z, matrix, userXRotationDeg);
    }

    /// <summary>
    /// Calculates the signed yaw offset, in degrees, from the physical MID-360
    /// +X axis after leveling to a user-selected X0 direction.  Both directions
    /// are constrained to the plane perpendicular to Project Z, so X0 is always
    /// orthogonal to Z0.  Positive angle follows the right-hand rule about Z0.
    /// </summary>
    public static double CalculateX0YawFromSensorX(Vector3 projectZInSensor, Vector3 desiredXInSensor)
    {
        if (!IsFinite(projectZInSensor) || projectZInSensor.LengthSquared() < 1e-8f)
            throw new ArgumentException("Project Z direction is invalid.", nameof(projectZInSensor));
        if (!IsFinite(desiredXInSensor) || desiredXInSensor.LengthSquared() < 1e-8f)
            throw new ArgumentException("Desired X direction is invalid.", nameof(desiredXInSensor));

        var z = Vector3.Normalize(projectZInSensor);

        // Physical MID-360 +X expressed in Sensor coordinates.
        var baseX = Vector3.UnitX - Vector3.Dot(Vector3.UnitX, z) * z;
        if (baseX.LengthSquared() < 1e-6f)
        {
            // Extremely unusual orientation: sensor +X is almost parallel to Z0.
            // Fall back to +Y only to keep the frame mathematically well-defined.
            baseX = Vector3.UnitY - Vector3.Dot(Vector3.UnitY, z) * z;
        }
        baseX = Vector3.Normalize(baseX);

        var desired = desiredXInSensor - Vector3.Dot(desiredXInSensor, z) * z;
        if (desired.LengthSquared() < 1e-6f)
            throw new ArgumentException("Desired X direction is parallel to Project Z.", nameof(desiredXInSensor));
        desired = Vector3.Normalize(desired);

        var sin = Vector3.Dot(z, Vector3.Cross(baseX, desired));
        var cos = Math.Clamp(Vector3.Dot(baseX, desired), -1f, 1f);
        return Math.Atan2(sin, cos) * 180.0 / Math.PI;
    }

    public Point3D ToWorld(Point3D sensorPoint)
    {
        var state = Volatile.Read(ref _state);
        return TransformPoint(sensorPoint, state.Forward);
    }

    public Point3D ToSensor(Point3D worldPoint)
    {
        var state = Volatile.Read(ref _state);
        return TransformPoint(worldPoint, state.Inverse);
    }

    public PointCloudFrame ToWorld(PointCloudFrame sensorFrame)
    {
        var state = Volatile.Read(ref _state);
        var source = sensorFrame.Points;
        var world = new Point3D[source.Count];
        for (var i = 0; i < source.Count; i++)
            world[i] = TransformPoint(source[i], state.Forward);
        return sensorFrame with { Points = world };
    }

    public void SetTransform(Matrix4x4 matrix, bool enforceRotationOnly = true)
    {
        if (!IsFinite(matrix)) throw new ArgumentException("Transform contains NaN/Infinity.", nameof(matrix));

        if (enforceRotationOnly)
        {
            matrix.M14 = matrix.M24 = matrix.M34 = 0;
            matrix.M41 = matrix.M42 = matrix.M43 = 0;
            matrix.M44 = 1;

            var x = new Vector3(matrix.M11, matrix.M21, matrix.M31);
            var y = new Vector3(matrix.M12, matrix.M22, matrix.M32);
            var z = new Vector3(matrix.M13, matrix.M23, matrix.M33);
            if (x.LengthSquared() < 1e-8f || y.LengthSquared() < 1e-8f || z.LengthSquared() < 1e-8f)
                throw new ArgumentException("Rotation axes are invalid.", nameof(matrix));

            z = Vector3.Normalize(z);
            x -= Vector3.Dot(x, z) * z;
            if (x.LengthSquared() < 1e-8f) throw new ArgumentException("X axis is parallel to Z axis.", nameof(matrix));
            x = Vector3.Normalize(x);
            y = Vector3.Normalize(Vector3.Cross(z, x));
            x = Vector3.Normalize(Vector3.Cross(y, z));
            matrix = CreateSensorToProjectRotation(x, y, z);
        }

        Volatile.Write(ref _state, CreateState(matrix));
    }

    public double RangeErrorMeters(Point3D sensor, Point3D project)
    {
        var rs = Math.Sqrt(sensor.X * sensor.X + sensor.Y * sensor.Y + sensor.Z * sensor.Z);
        var rp = Math.Sqrt(project.X * project.X + project.Y * project.Y + project.Z * project.Z);
        return Math.Abs(rs - rp);
    }

    public static double[] ToRowMajorArray(Matrix4x4 m) => new double[]
    {
        m.M11,m.M12,m.M13,m.M14, m.M21,m.M22,m.M23,m.M24,
        m.M31,m.M32,m.M33,m.M34, m.M41,m.M42,m.M43,m.M44
    };

    private static Point3D TransformPoint(Point3D p0, Matrix4x4 m)
    {
        var p = Vector3.Transform(new Vector3(p0.X, p0.Y, p0.Z), m);
        return new Point3D(p.X, p.Y, p.Z, p0.Reflectivity, p0.Tag);
    }

    private static TransformState CreateState(Matrix4x4 matrix)
    {
        if (!Matrix4x4.Invert(matrix, out var inverse))
            throw new ArgumentException("Transform is not invertible.", nameof(matrix));
        return new TransformState(
            matrix,
            inverse,
            new Vector3(matrix.M11, matrix.M21, matrix.M31),
            new Vector3(matrix.M12, matrix.M22, matrix.M32),
            new Vector3(matrix.M13, matrix.M23, matrix.M33));
    }

    private static Matrix4x4 CreateSensorToProjectRotation(Vector3 x, Vector3 y, Vector3 z)
    {
        return new Matrix4x4(
            x.X, y.X, z.X, 0,
            x.Y, y.Y, z.Y, 0,
            x.Z, y.Z, z.Z, 0,
            0,   0,   0,   1);
    }

    private static Vector3 RotateAroundAxis(Vector3 v, Vector3 axis, float angle)
    {
        axis = Vector3.Normalize(axis);
        var c = MathF.Cos(angle);
        var s = MathF.Sin(angle);
        return v * c + Vector3.Cross(axis, v) * s + axis * Vector3.Dot(axis, v) * (1f - c);
    }

    private static bool IsFinite(Vector3 v) =>
        float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);

    private static bool IsFinite(Matrix4x4 m)
    {
        Span<float> values = stackalloc float[16]
        {
            m.M11,m.M12,m.M13,m.M14,m.M21,m.M22,m.M23,m.M24,
            m.M31,m.M32,m.M33,m.M34,m.M41,m.M42,m.M43,m.M44
        };
        for (var i = 0; i < values.Length; i++)
            if (!float.IsFinite(values[i])) return false;
        return true;
    }
}

public readonly record struct ProjectFrameResult(
    Vector3 XAxisInSensor,
    Vector3 YAxisInSensor,
    Vector3 ZAxisInSensor,
    Matrix4x4 SensorToProject,
    double UserXRotationDeg);
