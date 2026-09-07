namespace LivoxHmi.Core;

public static class SurfaceGeometry
{
    public static (double U, double V, double N) Project(Point3D p, SurfaceDefinition s)
    {
        if (s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.OcctBSpline)
            return OcctSurfaceBridge.Project(p, s);
        return s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.Cylinder
            ? ProjectCylinder(p, s) : ProjectPlane(p, s);
    }

    private static (double U, double V, double N) ProjectPlane(Point3D p, SurfaceDefinition s)
    {
        var dx = p.X - s.Origin.X;
        var dy = p.Y - s.Origin.Y;
        var dz = p.Z - s.Origin.Z;
        var u = dx * s.UAxis.X + dy * s.UAxis.Y + dz * s.UAxis.Z;
        var v = dx * s.VAxis.X + dy * s.VAxis.Y + dz * s.VAxis.Z;
        var n = dx * s.Normal.X + dy * s.Normal.Y + dz * s.Normal.Z;
        return (u, v, n);
    }

    private static (double U, double V, double N) ProjectCylinder(Point3D p, SurfaceDefinition s)
    {
        var r = Math.Max(1e-6, s.CurvatureRadiusMeters);
        var sign = s.CurvatureSign < 0 ? -1.0 : 1.0;
        var ux = (double)s.UAxis.X; var uy = (double)s.UAxis.Y; var uz = (double)s.UAxis.Z;
        var vx = (double)s.VAxis.X; var vy = (double)s.VAxis.Y; var vz = (double)s.VAxis.Z;
        var nx = (double)s.Normal.X; var ny = (double)s.Normal.Y; var nz = (double)s.Normal.Z;
        Normalize(ref ux, ref uy, ref uz); Normalize(ref vx, ref vy, ref vz); Normalize(ref nx, ref ny, ref nz);

        // Axis line passes through C0 = Origin - Normal * radius and follows +V.
        var cx = s.Origin.X - sign * nx * r;
        var cy = s.Origin.Y - sign * ny * r;
        var cz = s.Origin.Z - sign * nz * r;
        var qx = p.X - cx; var qy = p.Y - cy; var qz = p.Z - cz;
        var v = qx * vx + qy * vy + qz * vz;
        var rx = qx - v * vx; var ry = qy - v * vy; var rz = qz - v * vz;
        var rho = Math.Sqrt(rx * rx + ry * ry + rz * rz);
        if (rho < 1e-9) return (0, v, -r);
        rx /= rho; ry /= rho; rz /= rho;
        var cos = rx * (sign * nx) + ry * (sign * ny) + rz * (sign * nz);
        var sin = rx * ux + ry * uy + rz * uz;
        var angle = Math.Atan2(sin, cos);
        if (angle < 0) angle += Math.PI * 2.0;
        var sweep = Math.Min(Math.PI * 2.0, Math.Max(0.0, s.WidthMeters / r));
        // For patches smaller than a full cylinder, prefer the equivalent angle closest to the patch.
        if (angle > sweep && angle - Math.PI * 2.0 >= -1e-9) angle -= Math.PI * 2.0;
        var u = angle * r;
        var n = rho - r; // signed radial distance from the cylindrical surface
        return (u, v, n);
    }

    public static Point3D FromUv(SurfaceDefinition s, double u, double v)
    {
        if (s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.OcctBSpline)
            return OcctSurfaceBridge.FromUv(s, u, v);
        return s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.Cylinder
            ? FromUvCylinder(s, u, v) : FromUvPlane(s, u, v);
    }

    private static Point3D FromUvPlane(SurfaceDefinition s, double u, double v)
    {
        double ux = s.UAxis.X; double uy = s.UAxis.Y; double uz = s.UAxis.Z;
        double vx = s.VAxis.X; double vy = s.VAxis.Y; double vz = s.VAxis.Z;
        Normalize(ref ux, ref uy, ref uz); Normalize(ref vx, ref vy, ref vz);
        return new Point3D(
            (float)(s.Origin.X + u * ux + v * vx),
            (float)(s.Origin.Y + u * uy + v * vy),
            (float)(s.Origin.Z + u * uz + v * vz), 0, 0);
    }

    private static Point3D FromUvCylinder(SurfaceDefinition s, double u, double v)
    {
        var r = Math.Max(1e-6, s.CurvatureRadiusMeters);
        var sign = s.CurvatureSign < 0 ? -1.0 : 1.0;
        double ux = s.UAxis.X; double uy = s.UAxis.Y; double uz = s.UAxis.Z;
        double vx = s.VAxis.X; double vy = s.VAxis.Y; double vz = s.VAxis.Z;
        double nx = s.Normal.X; double ny = s.Normal.Y; double nz = s.Normal.Z;
        Normalize(ref ux, ref uy, ref uz); Normalize(ref vx, ref vy, ref vz); Normalize(ref nx, ref ny, ref nz);
        var a = u / r;
        var ca = Math.Cos(a); var sa = Math.Sin(a);
        var cx = s.Origin.X - sign * nx * r;
        var cy = s.Origin.Y - sign * ny * r;
        var cz = s.Origin.Z - sign * nz * r;
        var rx = sign * nx * ca + ux * sa;
        var ry = sign * ny * ca + uy * sa;
        var rz = sign * nz * ca + uz * sa;
        return new Point3D((float)(cx + rx * r + vx * v), (float)(cy + ry * r + vy * v), (float)(cz + rz * r + vz * v), 0, 0);
    }

    public static Point3D NormalAtUv(SurfaceDefinition s, double u, double v)
    {
        if (s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.OcctBSpline)
            return OcctSurfaceBridge.NormalAtUv(s, u, v);
        if (s.Type != SurfaceType.Curved || s.CurvedKind != CurvedSurfaceKind.Cylinder)
            return NormalizePoint(s.Normal);
        var r = Math.Max(1e-6, s.CurvatureRadiusMeters);
        var sign = s.CurvatureSign < 0 ? -1.0 : 1.0;
        double ux = s.UAxis.X; double uy = s.UAxis.Y; double uz = s.UAxis.Z;
        double nx = s.Normal.X; double ny = s.Normal.Y; double nz = s.Normal.Z;
        Normalize(ref ux, ref uy, ref uz); Normalize(ref nx, ref ny, ref nz);
        var a = u / r; var ca = Math.Cos(a); var sa = Math.Sin(a);
        return new Point3D((float)(sign * nx * ca + ux * sa), (float)(sign * ny * ca + uy * sa), (float)(sign * nz * ca + uz * sa), 0, 0);
    }

    public static double RoundTripErrorMeters(SurfaceDefinition s, UvPoint uv)
    {
        var p = FromUv(s, uv.U, uv.V);
        var (u2, v2, n2) = Project(p, s);
        var du = u2 - uv.U; var dv = v2 - uv.V;
        return Math.Sqrt(du * du + dv * dv + n2 * n2);
    }

    public static bool IsUvInsideSurface(SurfaceDefinition s, double u, double v, double tolerance = 1e-6)
        => u >= -tolerance && v >= -tolerance && u <= s.WidthMeters + tolerance && v <= s.HeightMeters + tolerance;

    public static bool IsInside(ZoneDefinition zone, double u, double v)
    {
        if (!zone.Enabled || zone.Polygon.Count < 3) return false;
        bool inside = false;
        for (int i = 0, j = zone.Polygon.Count - 1; i < zone.Polygon.Count; j = i++)
        {
            var pi = zone.Polygon[i]; var pj = zone.Polygon[j];
            var intersect = ((pi.V > v) != (pj.V > v)) &&
                            (u < (pj.U - pi.U) * (v - pi.V) /
                                 ((pj.V - pi.V) == 0 ? double.Epsilon : (pj.V - pi.V)) + pi.U);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    // Ray is expressed in MID360_SENSOR coordinates. The test is deliberately two-sided.
    public static bool TryIntersectRay(SurfaceDefinition s, Point3D rayOrigin, Point3D rayDirection,
        out Point3D sensorPoint, out UvPoint uv)
    {
        sensorPoint = default; uv = default;
        if (s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.OcctBSpline)
            return OcctSurfaceBridge.TryIntersectRay(s, rayOrigin, rayDirection, out sensorPoint, out uv);
        return s.Type == SurfaceType.Curved && s.CurvedKind == CurvedSurfaceKind.Cylinder
            ? TryIntersectCylinder(s, rayOrigin, rayDirection, out sensorPoint, out uv)
            : TryIntersectPlane(s, rayOrigin, rayDirection, out sensorPoint, out uv);
    }

    private static bool TryIntersectPlane(SurfaceDefinition s, Point3D ro, Point3D rd, out Point3D p, out UvPoint uv)
    {
        p = default; uv = default;
        var nx = (double)s.Normal.X; var ny = (double)s.Normal.Y; var nz = (double)s.Normal.Z; Normalize(ref nx, ref ny, ref nz);
        var dx = (double)rd.X; var dy = (double)rd.Y; var dz = (double)rd.Z; Normalize(ref dx, ref dy, ref dz);
        var denom = dx * nx + dy * ny + dz * nz;
        if (Math.Abs(denom) < 1e-9) return false;
        var t = ((s.Origin.X - ro.X) * nx + (s.Origin.Y - ro.Y) * ny + (s.Origin.Z - ro.Z) * nz) / denom;
        if (t < 0) return false;
        p = new Point3D((float)(ro.X + t * dx), (float)(ro.Y + t * dy), (float)(ro.Z + t * dz), 0, 0);
        var pr = ProjectPlane(p, s); uv = new UvPoint(pr.U, pr.V);
        return IsUvInsideSurface(s, uv.U, uv.V, 0.002) && Math.Abs(pr.N) <= 0.003;
    }

    private static bool TryIntersectCylinder(SurfaceDefinition s, Point3D ro, Point3D rd, out Point3D p, out UvPoint uv)
    {
        p = default; uv = default;
        var r = Math.Max(1e-6, s.CurvatureRadiusMeters);
        var sign = s.CurvatureSign < 0 ? -1.0 : 1.0;
        double vx = s.VAxis.X, vy = s.VAxis.Y, vz = s.VAxis.Z; Normalize(ref vx, ref vy, ref vz);
        double nx = s.Normal.X, ny = s.Normal.Y, nz = s.Normal.Z; Normalize(ref nx, ref ny, ref nz);
        var cx = s.Origin.X - sign * nx * r; var cy = s.Origin.Y - sign * ny * r; var cz = s.Origin.Z - sign * nz * r;
        double dx = rd.X, dy = rd.Y, dz = rd.Z; Normalize(ref dx, ref dy, ref dz);
        var qx = ro.X - cx; var qy = ro.Y - cy; var qz = ro.Z - cz;
        var ddv = dx * vx + dy * vy + dz * vz;
        var qdv = qx * vx + qy * vy + qz * vz;
        var dpx = dx - ddv * vx; var dpy = dy - ddv * vy; var dpz = dz - ddv * vz;
        var qpx = qx - qdv * vx; var qpy = qy - qdv * vy; var qpz = qz - qdv * vz;
        var A = dpx * dpx + dpy * dpy + dpz * dpz;
        var B = 2.0 * (qpx * dpx + qpy * dpy + qpz * dpz);
        var C = qpx * qpx + qpy * qpy + qpz * qpz - r * r;
        if (A < 1e-12) return false;
        var disc = B * B - 4 * A * C;
        if (disc < 0) return false;
        var root = Math.Sqrt(Math.Max(0, disc));
        var t0 = (-B - root) / (2 * A); var t1 = (-B + root) / (2 * A);
        var candidates = new[] { t0, t1 }.Where(t => t >= 0).OrderBy(t => t);
        foreach (var t in candidates)
        {
            var hp = new Point3D((float)(ro.X + t * dx), (float)(ro.Y + t * dy), (float)(ro.Z + t * dz), 0, 0);
            var pr = ProjectCylinder(hp, s);
            if (!IsUvInsideSurface(s, pr.U, pr.V, 0.002) || Math.Abs(pr.N) > 0.004) continue;
            p = hp; uv = new UvPoint(pr.U, pr.V); return true;
        }
        return false;
    }



    public static double ApproxSurfacePathLength(SurfaceDefinition s, UvPoint a, UvPoint b, int segments = 24)
    {
        segments = Math.Clamp(segments, 1, 256);
        var prev = FromUv(s, a.U, a.V);
        double total = 0;
        for (var i = 1; i <= segments; i++)
        {
            var t = (double)i / segments;
            var p = FromUv(s, a.U + (b.U-a.U)*t, a.V + (b.V-a.V)*t);
            var dx=(double)p.X-prev.X; var dy=(double)p.Y-prev.Y; var dz=(double)p.Z-prev.Z;
            total += Math.Sqrt(dx*dx+dy*dy+dz*dz);
            prev = p;
        }
        return total;
    }

    public static bool IsOcctAvailable(out string status) => OcctSurfaceBridge.IsAvailable(out status);
    private static void Normalize(ref double x, ref double y, ref double z)
    {
        var len = Math.Sqrt(x * x + y * y + z * z);
        if (len < 1e-12) throw new InvalidOperationException("Degenerate surface basis.");
        x /= len; y /= len; z /= len;
    }

    private static Point3D NormalizePoint(Point3D p)
    {
        double x = p.X, y = p.Y, z = p.Z; Normalize(ref x, ref y, ref z);
        return new Point3D((float)x, (float)y, (float)z, 0, 0);
    }
}
