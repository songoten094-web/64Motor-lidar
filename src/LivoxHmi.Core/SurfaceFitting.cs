using System.Numerics;

namespace LivoxHmi.Core;

public sealed record OcctSurfaceFitResult(
    int SourcePoints,
    int ControlPoints,
    double RmseMm,
    double MaxAbsMm,
    string Message);

/// <summary>
/// Fits an OCCT B-Spline control grid to the fixed-sensor static map. The caller first
/// places/orients a coarse rectangular surface. That rectangle is the selection footprint;
/// no viewer pixels, camera coordinates or IMU transform enter this class.
/// </summary>
public static class OcctStaticMapFitter
{
    public static OcctSurfaceFitResult Fit(
        SurfaceDefinition surface,
        StaticMapSnapshot map,
        int gridU = 6,
        int gridV = 6,
        double searchDepthMeters = 0.30)
    {
        if (surface.Frame != "MID360_SENSOR")
            throw new InvalidOperationException("Surface phải ở MID360_SENSOR.");
        if (surface.Type != SurfaceType.Curved || surface.CurvedKind != CurvedSurfaceKind.OcctBSpline)
            throw new InvalidOperationException("FIT OCCT chỉ dùng cho OCCT B-Spline surface.");
        if (map.Points.Count == 0) throw new InvalidOperationException("Static map đang rỗng.");
        gridU = Math.Clamp(gridU, 4, 12); gridV = Math.Clamp(gridV, 4, 12);
        searchDepthMeters = Math.Clamp(searchDepthMeters, 0.03, 1.0);

        var o = V(surface.Origin); var uAxis = Vector3.Normalize(V(surface.UAxis));
        var vAxis = Vector3.Normalize(V(surface.VAxis)); var nAxis = Vector3.Normalize(V(surface.Normal));
        var width = Math.Max(0.02, surface.WidthMeters); var height = Math.Max(0.02, surface.HeightMeters);

        // Collect only static-map samples within the coarse surface footprint/prism.
        var samples = new List<(double u,double v,double n,Point3D p)>();
        foreach (var p in map.Points)
        {
            var q = V(p) - o;
            var u = Vector3.Dot(q,uAxis); var v = Vector3.Dot(q,vAxis); var n = Vector3.Dot(q,nAxis);
            if (u < -0.03 || u > width + 0.03 || v < -0.03 || v > height + 0.03) continue;
            if (Math.Abs(n) > searchDepthMeters) continue;
            samples.Add((u,v,n,p));
        }
        if (samples.Count < Math.Max(80, gridU*gridV*2))
            throw new InvalidOperationException($"Không đủ static-map point trong footprint ({samples.Count}). Hãy đặt Surface gần vùng cần fit hoặc tăng Fit depth.");

        var du = width/(gridU-1); var dv = height/(gridV-1);
        var offsets = new double[gridU,gridV];
        var valid = new bool[gridU,gridV];
        var radiusU = Math.Max(du*0.90, map.VoxelSizeMeters*2.5);
        var radiusV = Math.Max(dv*0.90, map.VoxelSizeMeters*2.5);

        // Robust local median normal offset at each grid node.
        for (var j=0;j<gridV;j++) for (var i=0;i<gridU;i++)
        {
            var tu=i*du; var tv=j*dv;
            var vals = new List<double>();
            foreach (var s in samples)
                if (Math.Abs(s.u-tu)<=radiusU && Math.Abs(s.v-tv)<=radiusV) vals.Add(s.n);
            if (vals.Count < 3) continue;
            vals.Sort(); offsets[i,j] = Median(vals); valid[i,j] = true;
        }

        // Fill sparse nodes from nearest valid node; fail if the map did not cover enough area.
        var validCount=valid.Cast<bool>().Count(x=>x);
        if (validCount < gridU*gridV/3)
            throw new InvalidOperationException($"Map chỉ phủ {validCount}/{gridU*gridV} fit nodes. Hãy chỉnh footprint Surface phủ đúng vùng cong.");
        for(var j=0;j<gridV;j++) for(var i=0;i<gridU;i++) if(!valid[i,j])
        {
            double best=double.MaxValue, val=0;
            for(var y=0;y<gridV;y++) for(var x=0;x<gridU;x++) if(valid[x,y])
            { var d=(x-i)*(x-i)+(y-j)*(y-j); if(d<best){best=d;val=offsets[x,y];} }
            offsets[i,j]=val;
        }

        // Gentle 3x3 smoothing. Preserve sensor-frame position; only local N offsets move.
        for(var pass=0;pass<2;pass++)
        {
            var next=(double[,])offsets.Clone();
            for(var j=0;j<gridV;j++) for(var i=0;i<gridU;i++)
            {
                double sum=0,w=0;
                for(var y=Math.Max(0,j-1);y<=Math.Min(gridV-1,j+1);y++)
                for(var x=Math.Max(0,i-1);x<=Math.Min(gridU-1,i+1);x++)
                { var ww=(x==i&&y==j)?3.0:1.0; sum+=offsets[x,y]*ww; w+=ww; }
                next[i,j]=sum/w;
            }
            offsets=next;
        }

        var control = new List<Point3D>(gridU*gridV);
        for(var j=0;j<gridV;j++) for(var i=0;i<gridU;i++)
        {
            var pos=o+uAxis*(float)(i*du)+vAxis*(float)(j*dv)+nAxis*(float)offsets[i,j];
            control.Add(new Point3D(pos.X,pos.Y,pos.Z,0,0));
        }
        surface.OcctFitUCount=gridU; surface.OcctFitVCount=gridV;
        surface.OcctFitControlPoints=control; surface.OcctFitSourcePointCount=samples.Count;

        // Validate against the generated OCCT surface. Robustly subsample to bound CPU.
        double sse=0,max=0; int used=0;
        var stride=Math.Max(1,samples.Count/5000);
        for(var k=0;k<samples.Count;k+=stride)
        {
            var pr=SurfaceGeometry.Project(samples[k].p,surface);
            if(!SurfaceGeometry.IsUvInsideSurface(surface,pr.U,pr.V,0.04)) continue;
            var e=Math.Abs(pr.N); sse+=e*e; max=Math.Max(max,e); used++;
        }
        var rmse=used>0?Math.Sqrt(sse/used):double.NaN;
        surface.OcctFitRmseMm=rmse*1000.0; surface.ValidationRmseMm=surface.OcctFitRmseMm;
        return new OcctSurfaceFitResult(samples.Count,control.Count,rmse*1000.0,max*1000.0,
            $"FIT OK | map {samples.Count:N0} pts | grid {gridU}×{gridV} | RMSE {rmse*1000.0:0.0} mm");
    }

    private static double Median(List<double> a) => a.Count%2==1?a[a.Count/2]:(a[a.Count/2-1]+a[a.Count/2])*0.5;
    private static Vector3 V(Point3D p)=>new(p.X,p.Y,p.Z);
}
