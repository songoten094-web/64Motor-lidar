using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;

namespace LivoxHmi.Core;

internal static class OcctSurfaceBridge
{
    private const string Dll = "OcctHmiBridge";
    private sealed class CacheEntry
    {
        public required IntPtr Handle;
        public required double U0;
        public required double U1;
        public required double V0;
        public required double V1;
    }
    private static readonly ConcurrentDictionary<string, CacheEntry> Cache = new();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)] private static extern int OcctHmi_GetApiVersion();
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)] private static extern IntPtr OcctHmi_CreateBsplineSurface(double[] xyz, int uCount, int vCount, double tolerance);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)] private static extern void OcctHmi_DestroySurface(IntPtr handle);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)] private static extern int OcctHmi_GetBounds(IntPtr handle, out double u0, out double u1, out double v0, out double v1);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)] private static extern int OcctHmi_Evaluate(IntPtr handle, double u, double v, double[] xyz, double[] normal);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)] private static extern int OcctHmi_Project(IntPtr handle, double[] xyz, out double u, out double v, double[] nearestXyz, out double signedDistance);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)] private static extern int OcctHmi_IntersectRay(IntPtr handle, double[] ro, double[] rd, double[] xyz, out double u, out double v, out double rayT);

    public static bool IsAvailable(out string status)
    {
        try { var v = OcctHmi_GetApiVersion(); status = $"OCCT bridge API {v}"; return v >= 1960; }
        catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
        { status = "OCCT bridge chưa build/load: " + ex.GetType().Name; return false; }
    }

    private static CacheEntry Get(SurfaceDefinition s)
    {
        var key = GeometryKey(s);
        return Cache.GetOrAdd(key, _ => Create(s));
    }

    private static CacheEntry Create(SurfaceDefinition s)
    {
        var nu = s.OcctFitUCount >= 2 && s.OcctFitControlPoints.Count == s.OcctFitUCount * s.OcctFitVCount ? s.OcctFitUCount : 4;
        var nv = s.OcctFitVCount >= 2 && s.OcctFitControlPoints.Count == s.OcctFitUCount * s.OcctFitVCount ? s.OcctFitVCount : 4;
        var xyz = BuildControlGrid(s, nu, nv);
        var h = OcctHmi_CreateBsplineSurface(xyz, nu, nv, 1e-5);
        if (h == IntPtr.Zero) throw new InvalidOperationException("OCCT không tạo được B-Spline Surface.");
        if (OcctHmi_GetBounds(h, out var u0, out var u1, out var v0, out var v1) == 0)
        { OcctHmi_DestroySurface(h); throw new InvalidOperationException("OCCT GetBounds failed."); }
        return new CacheEntry { Handle=h, U0=u0, U1=u1, V0=v0, V1=v1 };
    }

    private static double[] BuildControlGrid(SurfaceDefinition s, int nu, int nv)
    {
        if (s.OcctFitUCount >= 2 && s.OcctFitVCount >= 2 &&
            s.OcctFitControlPoints.Count == s.OcctFitUCount * s.OcctFitVCount)
        {
            nu = s.OcctFitUCount; nv = s.OcctFitVCount;
            var fitted = new double[nu * nv * 3];
            for (var i = 0; i < s.OcctFitControlPoints.Count; i++)
            {
                var p = s.OcctFitControlPoints[i]; var k = i * 3;
                fitted[k] = p.X; fitted[k + 1] = p.Y; fitted[k + 2] = p.Z;
            }
            return fitted;
        }
        var data = new double[nu*nv*3];
        var ux=(double)s.UAxis.X; var uy=(double)s.UAxis.Y; var uz=(double)s.UAxis.Z;
        var vx=(double)s.VAxis.X; var vy=(double)s.VAxis.Y; var vz=(double)s.VAxis.Z;
        var nx=(double)s.Normal.X; var ny=(double)s.Normal.Y; var nz=(double)s.Normal.Z;
        Normalize(ref ux,ref uy,ref uz); Normalize(ref vx,ref vy,ref vz); Normalize(ref nx,ref ny,ref nz);
        for (var j=0;j<nv;j++) for (var i=0;i<nu;i++)
        {
            var fu=(double)i/(nu-1); var fv=(double)j/(nv-1);
            var u=s.WidthMeters*fu; var v=s.HeightMeters*fv;
            // Smooth freeform deformation. Edge stays on the base rectangle, center receives bulge.
            var bulge=s.OcctBulgeMeters*Math.Sin(Math.PI*fu)*Math.Sin(Math.PI*fv);
            var twist=s.OcctTwistMeters*(2*fu-1)*(2*fv-1);
            var off=bulge+twist;
            var k=(j*nu+i)*3;
            data[k]=s.Origin.X+u*ux+v*vx+off*nx;
            data[k+1]=s.Origin.Y+u*uy+v*vy+off*ny;
            data[k+2]=s.Origin.Z+u*uz+v*vz+off*nz;
        }
        return data;
    }

    private static string GeometryKey(SurfaceDefinition s)
    {
        var sb = new StringBuilder();
        sb.Append(System.FormattableString.Invariant($"{s.Origin.X:R},{s.Origin.Y:R},{s.Origin.Z:R}|{s.UAxis.X:R},{s.UAxis.Y:R},{s.UAxis.Z:R}|{s.VAxis.X:R},{s.VAxis.Y:R},{s.VAxis.Z:R}|{s.Normal.X:R},{s.Normal.Y:R},{s.Normal.Z:R}|{s.WidthMeters:R},{s.HeightMeters:R},{s.OcctBulgeMeters:R},{s.OcctTwistMeters:R}|FIT={s.OcctFitUCount}x{s.OcctFitVCount}|"));
        foreach (var p in s.OcctFitControlPoints) sb.Append(System.FormattableString.Invariant($"{p.X:R},{p.Y:R},{p.Z:R};"));
        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(sb.ToString())));
    }

    private static (double u,double v) AppToNative(CacheEntry h, SurfaceDefinition s, double u, double v)
        => (h.U0+(h.U1-h.U0)*(u/Math.Max(1e-9,s.WidthMeters)), h.V0+(h.V1-h.V0)*(v/Math.Max(1e-9,s.HeightMeters)));
    private static UvPoint NativeToApp(CacheEntry h, SurfaceDefinition s, double u, double v)
        => new((u-h.U0)/Math.Max(1e-12,h.U1-h.U0)*s.WidthMeters, (v-h.V0)/Math.Max(1e-12,h.V1-h.V0)*s.HeightMeters);

    public static Point3D FromUv(SurfaceDefinition s, double u, double v)
    {
        var h=Get(s); var (un,vn)=AppToNative(h,s,u,v); var xyz=new double[3]; var n=new double[3];
        if (OcctHmi_Evaluate(h.Handle,un,vn,xyz,n)==0) throw new InvalidOperationException("OCCT Evaluate failed.");
        return new Point3D((float)xyz[0],(float)xyz[1],(float)xyz[2],0,0);
    }
    public static Point3D NormalAtUv(SurfaceDefinition s,double u,double v)
    {
        var h=Get(s); var (un,vn)=AppToNative(h,s,u,v); var xyz=new double[3]; var n=new double[3];
        if (OcctHmi_Evaluate(h.Handle,un,vn,xyz,n)==0) throw new InvalidOperationException("OCCT Evaluate failed.");
        return new Point3D((float)n[0],(float)n[1],(float)n[2],0,0);
    }
    public static (double U,double V,double N) Project(Point3D p,SurfaceDefinition s)
    {
        var h=Get(s); var q=new[]{(double)p.X,p.Y,p.Z}; var near=new double[3];
        if(OcctHmi_Project(h.Handle,q,out var un,out var vn,near,out var d)==0) throw new InvalidOperationException("OCCT Project failed.");
        var uv=NativeToApp(h,s,un,vn); return (uv.U,uv.V,d);
    }
    public static bool TryIntersectRay(SurfaceDefinition s,Point3D ro,Point3D rd,out Point3D p,out UvPoint uv)
    {
        p=default; uv=default; var h=Get(s); var q=new[]{(double)ro.X,ro.Y,ro.Z}; var d=new[]{(double)rd.X,rd.Y,rd.Z}; var hit=new double[3];
        if(OcctHmi_IntersectRay(h.Handle,q,d,hit,out var un,out var vn,out _)==0) return false;
        uv=NativeToApp(h,s,un,vn); p=new Point3D((float)hit[0],(float)hit[1],(float)hit[2],0,0);
        return SurfaceGeometry.IsUvInsideSurface(s,uv.U,uv.V,0.002);
    }

    private static void Normalize(ref double x,ref double y,ref double z){var l=Math.Sqrt(x*x+y*y+z*z);if(l<1e-12)throw new InvalidOperationException("Degenerate basis");x/=l;y/=l;z/=l;}
}
