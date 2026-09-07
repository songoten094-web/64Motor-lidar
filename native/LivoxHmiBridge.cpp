
#include "LivoxHmiBridge.h"
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <vector>
#include <algorithm>
#include <array>

namespace {

constexpr size_t kMaxPointsPerFrame = 120000;
constexpr uint64_t kFrameWindowUs = 50000; // 20 Hz application frame window.

struct FrameBuffer {
    std::vector<LivoxHmiPoint> points;
    uint64_t timestamp_us = 0;
    uint32_t dropped_points = 0;
    uint32_t packet_count = 0;
    uint32_t handle = 0;

    FrameBuffer() { points.reserve(kMaxPointsPerFrame); }
    void clear() {
        points.clear();
        timestamp_us = 0;
        dropped_points = 0;
        packet_count = 0;
        handle = 0;
    }
};

// Threading rule:
// 1) The Livox SDK callback is producer-only.
// 2) The C# polling thread is consumer-only.
// 3) All frame-buffer state is protected by one short-lived mutex.
// This deliberately favors bounded latency over an ever-growing queue.
std::atomic<bool> g_initialized{false};
std::atomic<bool> g_shutting_down{false};
std::atomic<uint64_t> g_packets_received{0};
std::atomic<uint64_t> g_packets_dropped{0};
std::atomic<uint32_t> g_active_handle{0};
std::atomic<bool> g_hms_query_in_flight{false};
std::atomic<int> g_hms_query_status{0}; // 0 idle/no data, 1 OK, negative = SDK/query error
std::array<uint32_t, 8> g_hms_codes{};
uint32_t g_hms_count = 0;
std::mutex g_hms_mutex;

struct ImuSample {
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;
    uint64_t timestamp_us = 0;
    uint32_t handle = 0;
};

constexpr size_t kImuWindowMaxSamples = 500; // ~2.5 s at 200 Hz
std::array<ImuSample, kImuWindowMaxSamples> g_imu_window{};
size_t g_imu_count = 0;
size_t g_imu_write_index = 0;
std::mutex g_imu_mutex;
std::atomic<uint64_t> g_imu_total_samples{0};

FrameBuffer g_building;
FrameBuffer g_ready;
std::mutex g_frame_mutex;
std::atomic<bool> g_ready_available{false};
uint64_t g_building_start_us = 0;

uint64_t now_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()).count();
}

void append_point_locked(float x_m, float y_m, float z_m, uint8_t refl, uint8_t tag,
                         uint32_t handle, uint64_t t) {
    if (g_building.points.size() >= kMaxPointsPerFrame) {
        ++g_building.dropped_points;
        g_packets_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    g_building.points.push_back({x_m, y_m, z_m, refl, tag, 0});
    g_building.handle = handle;
    g_building.timestamp_us = t;
}

void publish_if_needed_locked(uint64_t t) {
    if (g_building.points.empty()) return;
    if (g_building_start_us == 0) g_building_start_us = t;
    if (t - g_building_start_us < kFrameWindowUs) return;

    // Only one ready frame is kept. If the UI/consumer is slower, the old
    // frame is discarded instead of allowing latency and memory usage to grow.
    if (g_ready_available.load(std::memory_order_relaxed)) {
        g_packets_dropped.fetch_add(g_ready.packet_count, std::memory_order_relaxed);
    }

    std::swap(g_ready, g_building);
    g_ready_available.store(true, std::memory_order_release);
    g_building.clear();
    g_building_start_us = t;
}

void PointCloudCallback(uint32_t handle, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!data || g_shutting_down.load(std::memory_order_acquire)) return;

    g_packets_received.fetch_add(1, std::memory_order_relaxed);
    const uint64_t t = now_us();

    std::lock_guard<std::mutex> lock(g_frame_mutex);
    if (g_shutting_down.load(std::memory_order_relaxed)) return;
    ++g_building.packet_count;

    switch (data->data_type) {
    case kLivoxLidarCartesianCoordinateHighData: {
        auto* p = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);
        for (uint32_t i = 0; i < data->dot_num; ++i) {
            append_point_locked(p[i].x / 1000.0f, p[i].y / 1000.0f, p[i].z / 1000.0f,
                                p[i].reflectivity, p[i].tag, handle, t);
        }
        break;
    }
    case kLivoxLidarCartesianCoordinateLowData: {
        auto* p = reinterpret_cast<LivoxLidarCartesianLowRawPoint*>(data->data);
        for (uint32_t i = 0; i < data->dot_num; ++i) {
            append_point_locked(p[i].x / 100.0f, p[i].y / 100.0f, p[i].z / 100.0f,
                                p[i].reflectivity, p[i].tag, handle, t);
        }
        break;
    }
    case kLivoxLidarDoubleEchoData: {
        auto* p = reinterpret_cast<LivoxLidarDoubleEchoRawPoint*>(data->data);
        for (uint32_t i = 0; i < data->dot_num; ++i) {
            append_point_locked(p[i].x1 / 1000.0f, p[i].y1 / 1000.0f, p[i].z1 / 1000.0f,
                                p[i].reflectivity1, p[i].tag1, handle, t);
        }
        break;
    }
    default:
        // Spherical data is intentionally not converted in Step 1. MID-360 is
        // configured for Cartesian high data for a deterministic pipeline.
        break;
    }

    publish_if_needed_locked(t);
}

void ImuDataCallback(uint32_t handle, const uint8_t /*dev_type*/,
                     LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!data || g_shutting_down.load(std::memory_order_acquire)) return;
    if (data->data_type != kLivoxLidarImuData) return;

    const auto* p = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
    const uint64_t t = now_us();
    std::lock_guard<std::mutex> lock(g_imu_mutex);
    for (uint32_t i = 0; i < data->dot_num; ++i) {
        g_imu_window[g_imu_write_index] = {p[i].gyro_x, p[i].gyro_y, p[i].gyro_z,
                                           p[i].acc_x, p[i].acc_y, p[i].acc_z, t, handle};
        g_imu_write_index = (g_imu_write_index + 1) % kImuWindowMaxSamples;
        if (g_imu_count < kImuWindowMaxSamples) ++g_imu_count;
        g_imu_total_samples.fetch_add(1, std::memory_order_relaxed);
    }
    g_active_handle.store(handle, std::memory_order_release);
}

void HmsQueryCallback(livox_status status, uint32_t handle,
                      LivoxLidarDiagInternalInfoResponse* response, void* /*client_data*/) {
    g_hms_query_in_flight.store(false, std::memory_order_release);
    if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
        g_hms_query_status.store(status == kLivoxLidarStatusSuccess ? -100 - (response ? response->ret_code : 1) : static_cast<int>(status));
        return;
    }

    std::array<uint32_t, 8> latest{};
    uint32_t latest_count = 0;
    uint16_t off = 0;
    for (uint16_t i = 0; i < response->param_num; ++i) {
        auto* kv = reinterpret_cast<LivoxLidarKeyValueParam*>(&response->data[off]);
        if (kv->key == kKeyHmsCode) {
            const uint32_t available = std::min<uint32_t>(8, kv->length / sizeof(uint32_t));
            if (available > 0) {
                std::memcpy(latest.data(), kv->value, available * sizeof(uint32_t));
                latest_count = available;
            }
        }
        off = static_cast<uint16_t>(off + sizeof(uint16_t) * 2 + kv->length);
    }

    {
        std::lock_guard<std::mutex> lock(g_hms_mutex);
        g_hms_codes = latest;
        g_hms_count = latest_count;
    }
    g_active_handle.store(handle, std::memory_order_release);
    g_hms_query_status.store(1, std::memory_order_release);
}

void LidarInfoChangeCallback(uint32_t handle, const LivoxLidarInfo* info,
                             void* /*client_data*/) {
    if (!info || g_shutting_down.load(std::memory_order_acquire)) return;

    // Keep device state deterministic for the viewer. This remains entirely
    // inside the vendor SDK callback boundary; the SDK itself is not modified.
    g_active_handle.store(handle, std::memory_order_release);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    SetLivoxLidarPclDataType(handle, kLivoxLidarCartesianCoordinateHighData,
                             nullptr, nullptr);
    // MID-360 IMU is enabled through the vendor SDK. The callback is kept
    // independent from point-cloud rendering so UI stalls cannot block IMU.
    EnableLivoxLidarImuData(handle, nullptr, nullptr);
    if (!g_hms_query_in_flight.exchange(true))
        QueryLivoxLidarInternalInfo(handle, HmsQueryCallback, nullptr);
}

} // namespace

extern "C" {

int LivoxHmi_Initialize(const char* config_path) {
    if (!config_path || g_initialized.load()) return 0;
    g_shutting_down.store(false, std::memory_order_release);

    {
        std::lock_guard<std::mutex> lock(g_frame_mutex);
        g_building.clear();
        g_ready.clear();
        g_ready_available.store(false);
        g_building_start_us = 0;
    }

    g_packets_received.store(0);
    g_packets_dropped.store(0);
    g_active_handle.store(0);
    g_hms_query_in_flight.store(false);
    g_hms_query_status.store(0);
    {
        std::lock_guard<std::mutex> lock(g_hms_mutex);
        g_hms_codes.fill(0);
        g_hms_count = 0;
    }
    {
        std::lock_guard<std::mutex> lock(g_imu_mutex);
        g_imu_count = 0;
        g_imu_write_index = 0;
    }
    g_imu_total_samples.store(0);

    if (!LivoxLidarSdkInit(config_path)) {
        return 0;
    }

    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    g_initialized.store(true);
    return 1;
}

void LivoxHmi_Shutdown(void) {
    if (!g_initialized.exchange(false)) return;

    g_shutting_down.store(true, std::memory_order_release);
    LivoxLidarSdkUninit();

    std::lock_guard<std::mutex> lock(g_frame_mutex);
    g_building.clear();
    g_ready.clear();
    g_ready_available.store(false);
    g_active_handle.store(0);
    g_hms_query_in_flight.store(false);
    {
        std::lock_guard<std::mutex> imu_lock(g_imu_mutex);
        g_imu_count = 0;
        g_imu_write_index = 0;
    }
}

int LivoxHmi_IsInitialized(void) {
    return g_initialized.load() ? 1 : 0;
}

uint32_t LivoxHmi_GetQueueDepth(void) {
    return g_ready_available.load() ? 1u : 0u;
}

uint64_t LivoxHmi_GetPacketsReceived(void) {
    return g_packets_received.load();
}

uint64_t LivoxHmi_GetPacketsDropped(void) {
    return g_packets_dropped.load();
}


uint32_t LivoxHmi_GetApiVersion(void) {
    return 1602u;
}

int LivoxHmi_GetImuSnapshot(LivoxHmiImuSnapshot* out_snapshot) {
    if (!out_snapshot) return -1;
    std::memset(out_snapshot, 0, sizeof(LivoxHmiImuSnapshot));
    if (!g_initialized.load(std::memory_order_acquire)) return -2;

    std::lock_guard<std::mutex> lock(g_imu_mutex);
    if (g_imu_count == 0) return 0;

    const size_t latest_index = (g_imu_write_index + kImuWindowMaxSamples - 1) % kImuWindowMaxSamples;
    const auto& latest = g_imu_window[latest_index];
    out_snapshot->timestamp_us = latest.timestamp_us;
    out_snapshot->total_samples = g_imu_total_samples.load(std::memory_order_relaxed);
    out_snapshot->lidar_handle = latest.handle;
    out_snapshot->window_samples = static_cast<uint32_t>(g_imu_count);
    out_snapshot->gyro_x_rad_s = latest.gx;
    out_snapshot->gyro_y_rad_s = latest.gy;
    out_snapshot->gyro_z_rad_s = latest.gz;
    out_snapshot->acc_x_g = latest.ax;
    out_snapshot->acc_y_g = latest.ay;
    out_snapshot->acc_z_g = latest.az;

    double sgx=0, sgy=0, sgz=0, sax=0, say=0, saz=0;
    for (size_t i = 0; i < g_imu_count; ++i) {
        const auto& v = g_imu_window[i];
        sgx += v.gx; sgy += v.gy; sgz += v.gz;
        sax += v.ax; say += v.ay; saz += v.az;
    }
    const double n = static_cast<double>(g_imu_count);
    const double agx=sgx/n, agy=sgy/n, agz=sgz/n;
    const double aax=sax/n, aay=say/n, aaz=saz/n;
    out_snapshot->avg_gyro_x_rad_s = static_cast<float>(agx);
    out_snapshot->avg_gyro_y_rad_s = static_cast<float>(agy);
    out_snapshot->avg_gyro_z_rad_s = static_cast<float>(agz);
    out_snapshot->avg_acc_x_g = static_cast<float>(aax);
    out_snapshot->avg_acc_y_g = static_cast<float>(aay);
    out_snapshot->avg_acc_z_g = static_cast<float>(aaz);

    double acc_var = 0.0;
    double gyro_sq = 0.0;
    for (size_t i = 0; i < g_imu_count; ++i) {
        const auto& v = g_imu_window[i];
        const double dax=v.ax-aax, day=v.ay-aay, daz=v.az-aaz;
        acc_var += dax*dax + day*day + daz*daz;
        const double dgx=v.gx-agx, dgy=v.gy-agy, dgz=v.gz-agz;
        gyro_sq += dgx*dgx + dgy*dgy + dgz*dgz;
    }
    out_snapshot->accel_std_g = static_cast<float>(std::sqrt(acc_var / n));
    out_snapshot->gyro_rms_rad_s = static_cast<float>(std::sqrt(gyro_sq / n));
    return 1;
}

int LivoxHmi_RequestHms(void) {
    if (!g_initialized.load(std::memory_order_acquire)) return -2;
    const uint32_t handle = g_active_handle.load(std::memory_order_acquire);
    if (handle == 0) return -3;
    if (g_hms_query_in_flight.exchange(true, std::memory_order_acq_rel)) return 0;
    const livox_status rc = QueryLivoxLidarInternalInfo(handle, HmsQueryCallback, nullptr);
    if (rc != kLivoxLidarStatusSuccess) {
        g_hms_query_in_flight.store(false, std::memory_order_release);
        g_hms_query_status.store(static_cast<int>(rc), std::memory_order_release);
        return static_cast<int>(rc);
    }
    return 1;
}

uint32_t LivoxHmi_GetHmsCodes(uint32_t* out_codes, uint32_t capacity) {
    if (!out_codes || capacity == 0) return 0;
    std::lock_guard<std::mutex> lock(g_hms_mutex);
    const uint32_t n = std::min<uint32_t>(capacity, g_hms_count);
    if (n > 0) std::memcpy(out_codes, g_hms_codes.data(), n * sizeof(uint32_t));
    return n;
}

int LivoxHmi_GetHmsQueryStatus(void) {
    return g_hms_query_status.load(std::memory_order_acquire);
}

int LivoxHmi_TryGetFrame(LivoxHmiPoint* out_points,
                         uint32_t capacity,
                         LivoxHmiFrameInfo* out_info) {
    if (!out_points || !out_info || capacity == 0) return -1;

    std::lock_guard<std::mutex> lock(g_frame_mutex);
    if (!g_ready_available.load(std::memory_order_acquire)) return 0;
    g_ready_available.store(false, std::memory_order_release);

    const uint32_t count = static_cast<uint32_t>(
        std::min<size_t>(g_ready.points.size(), capacity));

    if (count > 0) {
        std::memcpy(out_points, g_ready.points.data(),
                    sizeof(LivoxHmiPoint) * count);
    }

    out_info->timestamp_us = g_ready.timestamp_us;
    out_info->point_count = count;
    out_info->dropped_points = g_ready.dropped_points +
                               static_cast<uint32_t>(g_ready.points.size() - count);
    out_info->packet_count = g_ready.packet_count;
    out_info->lidar_handle = g_ready.handle;

    g_ready.clear();
    return 1;
}

} // extern "C"
