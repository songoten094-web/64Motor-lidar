
#pragma once
#include <stdint.h>

#ifdef _WIN32
#define LIVOX_HMI_API __declspec(dllexport)
#else
#define LIVOX_HMI_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LivoxHmiPoint {
    float x_m;
    float y_m;
    float z_m;
    uint8_t reflectivity;
    uint8_t tag;
    uint16_t reserved;
} LivoxHmiPoint;


typedef struct LivoxHmiImuSnapshot {
    uint64_t timestamp_us;
    uint64_t total_samples;
    uint32_t lidar_handle;
    uint32_t window_samples;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float avg_gyro_x_rad_s;
    float avg_gyro_y_rad_s;
    float avg_gyro_z_rad_s;
    float avg_acc_x_g;
    float avg_acc_y_g;
    float avg_acc_z_g;
    float accel_std_g;
    float gyro_rms_rad_s;
} LivoxHmiImuSnapshot;

typedef struct LivoxHmiFrameInfo {
    uint64_t timestamp_us;
    uint32_t point_count;
    uint32_t dropped_points;
    uint32_t packet_count;
    uint32_t lidar_handle;
} LivoxHmiFrameInfo;

LIVOX_HMI_API int LivoxHmi_Initialize(const char* config_path);
LIVOX_HMI_API void LivoxHmi_Shutdown(void);
LIVOX_HMI_API int LivoxHmi_IsInitialized(void);
LIVOX_HMI_API uint32_t LivoxHmi_GetQueueDepth(void);
LIVOX_HMI_API uint64_t LivoxHmi_GetPacketsReceived(void);
LIVOX_HMI_API uint64_t LivoxHmi_GetPacketsDropped(void);
LIVOX_HMI_API uint32_t LivoxHmi_GetApiVersion(void);
LIVOX_HMI_API int LivoxHmi_GetImuSnapshot(LivoxHmiImuSnapshot* out_snapshot);
LIVOX_HMI_API int LivoxHmi_RequestHms(void);
LIVOX_HMI_API uint32_t LivoxHmi_GetHmsCodes(uint32_t* out_codes, uint32_t capacity);
LIVOX_HMI_API int LivoxHmi_GetHmsQueryStatus(void);
LIVOX_HMI_API int LivoxHmi_TryGetFrame(LivoxHmiPoint* out_points,
                                        uint32_t capacity,
                                        LivoxHmiFrameInfo* out_info);

#ifdef __cplusplus
}
#endif
