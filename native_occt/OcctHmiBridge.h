#pragma once
#ifdef _WIN32
#define OCCTHMI_API __declspec(dllexport)
#else
#define OCCTHMI_API
#endif
#include <cstdint>
extern "C" {
OCCTHMI_API int OcctHmi_GetApiVersion();
OCCTHMI_API void* OcctHmi_CreateBsplineSurface(const double* xyz, int uCount, int vCount, double tolerance);
OCCTHMI_API void OcctHmi_DestroySurface(void* handle);
OCCTHMI_API int OcctHmi_GetBounds(void* handle, double* u0, double* u1, double* v0, double* v1);
OCCTHMI_API int OcctHmi_Evaluate(void* handle, double u, double v, double* xyz, double* normal);
OCCTHMI_API int OcctHmi_Project(void* handle, const double* xyz, double* u, double* v, double* nearestXyz, double* signedDistance);
OCCTHMI_API int OcctHmi_IntersectRay(void* handle, const double* rayOrigin, const double* rayDirection, double* xyz, double* u, double* v, double* rayT);
}
