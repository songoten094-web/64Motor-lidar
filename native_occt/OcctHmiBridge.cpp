#include "OcctHmiBridge.h"
#include <Geom_BSplineSurface.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <Precision.hxx>
#include <GeomAbs_Shape.hxx>
#include <Standard_Failure.hxx>
#include <algorithm>
#include <cmath>
#include <limits>

struct SurfaceHandle {
    Handle(Geom_BSplineSurface) surface;
    TopoDS_Face face;
    double u0{}, u1{}, v0{}, v1{};
};

static bool valid(void* h) { return h != nullptr; }

extern "C" OCCTHMI_API int OcctHmi_GetApiVersion() { return 1960; }

extern "C" OCCTHMI_API void* OcctHmi_CreateBsplineSurface(const double* xyz, int uCount, int vCount, double tolerance) {
    if (!xyz || uCount < 2 || vCount < 2) return nullptr;
    try {
        TColgp_Array2OfPnt points(1, uCount, 1, vCount);
        for (int j = 0; j < vCount; ++j) {
            for (int i = 0; i < uCount; ++i) {
                const int k = (j * uCount + i) * 3;
                points.SetValue(i + 1, j + 1, gp_Pnt(xyz[k], xyz[k + 1], xyz[k + 2]));
            }
        }
        const double tol = std::max(1e-7, tolerance);
        GeomAPI_PointsToBSplineSurface builder(points, 3, 8, GeomAbs_C2, tol);
        if (!builder.IsDone()) return nullptr;
        Handle(Geom_BSplineSurface) s = builder.Surface();
        if (s.IsNull()) return nullptr;
        auto* h = new SurfaceHandle();
        h->surface = s;
        s->Bounds(h->u0, h->u1, h->v0, h->v1);
        h->face = BRepBuilderAPI_MakeFace(s, h->u0, h->u1, h->v0, h->v1, Precision::Confusion()).Face();
        return h;
    } catch (const Standard_Failure&) { return nullptr; }
      catch (...) { return nullptr; }
}

extern "C" OCCTHMI_API void OcctHmi_DestroySurface(void* handle) {
    delete static_cast<SurfaceHandle*>(handle);
}

extern "C" OCCTHMI_API int OcctHmi_GetBounds(void* handle, double* u0, double* u1, double* v0, double* v1) {
    if (!valid(handle) || !u0 || !u1 || !v0 || !v1) return 0;
    auto* h = static_cast<SurfaceHandle*>(handle);
    *u0 = h->u0; *u1 = h->u1; *v0 = h->v0; *v1 = h->v1;
    return 1;
}

extern "C" OCCTHMI_API int OcctHmi_Evaluate(void* handle, double u, double v, double* xyz, double* normal) {
    if (!valid(handle) || !xyz || !normal) return 0;
    try {
        auto* h = static_cast<SurfaceHandle*>(handle);
        gp_Pnt p; gp_Vec du, dv;
        h->surface->D1(u, v, p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (n.SquareMagnitude() < 1e-24) return 0;
        n.Normalize();
        xyz[0]=p.X(); xyz[1]=p.Y(); xyz[2]=p.Z();
        normal[0]=n.X(); normal[1]=n.Y(); normal[2]=n.Z();
        return 1;
    } catch (...) { return 0; }
}

extern "C" OCCTHMI_API int OcctHmi_Project(void* handle, const double* xyz, double* u, double* v, double* nearestXyz, double* signedDistance) {
    if (!valid(handle) || !xyz || !u || !v || !nearestXyz || !signedDistance) return 0;
    try {
        auto* h = static_cast<SurfaceHandle*>(handle);
        gp_Pnt q(xyz[0], xyz[1], xyz[2]);
        GeomAPI_ProjectPointOnSurf proj(q, h->surface, h->u0, h->u1, h->v0, h->v1);
        if (proj.NbPoints() < 1) return 0;
        proj.LowerDistanceParameters(*u, *v);
        gp_Pnt p = proj.NearestPoint();
        nearestXyz[0]=p.X(); nearestXyz[1]=p.Y(); nearestXyz[2]=p.Z();
        gp_Pnt eval; gp_Vec du,dv; h->surface->D1(*u,*v,eval,du,dv);
        gp_Vec n = du.Crossed(dv);
        if (n.SquareMagnitude() < 1e-24) return 0;
        n.Normalize();
        gp_Vec d(p, q);
        *signedDistance = d.Dot(n);
        return 1;
    } catch (...) { return 0; }
}

extern "C" OCCTHMI_API int OcctHmi_IntersectRay(void* handle, const double* rayOrigin, const double* rayDirection, double* xyz, double* u, double* v, double* rayT) {
    if (!valid(handle) || !rayOrigin || !rayDirection || !xyz || !u || !v || !rayT) return 0;
    try {
        auto* h = static_cast<SurfaceHandle*>(handle);
        gp_Vec d(rayDirection[0], rayDirection[1], rayDirection[2]);
        if (d.SquareMagnitude() < 1e-24) return 0;
        gp_Lin line(gp_Pnt(rayOrigin[0],rayOrigin[1],rayOrigin[2]), gp_Dir(d));
        IntCurvesFace_ShapeIntersector inter;
        inter.Load(h->face, 1e-7);
        inter.Perform(line, 0.0, 1.0e9);
        if (!inter.IsDone() || inter.NbPnt() < 1) return 0;
        double best = std::numeric_limits<double>::infinity(); int bestI = -1;
        for (int i=1;i<=inter.NbPnt();++i) {
            const double w = inter.WParameter(i);
            if (w >= 0.0 && w < best) { best = w; bestI = i; }
        }
        if (bestI < 0) return 0;
        gp_Pnt p = inter.Pnt(bestI);
        xyz[0]=p.X(); xyz[1]=p.Y(); xyz[2]=p.Z();
        *u=inter.UParameter(bestI); *v=inter.VParameter(bestI); *rayT=best;
        return 1;
    } catch (...) { return 0; }
}
