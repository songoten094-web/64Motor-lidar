
using System.Runtime.InteropServices;
using LivoxHmi.Core;

namespace LivoxHmi.Device;

internal static class LivoxNative
{
    private const string Dll = "LivoxHmiBridge";

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    internal struct NativePoint
    {
        public float X;
        public float Y;
        public float Z;
        public byte Reflectivity;
        public byte Tag;
        public ushort Reserved;
    }


    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    internal struct NativeImuSnapshot
    {
        public ulong TimestampUs;
        public ulong TotalSamples;
        public uint LidarHandle;
        public uint WindowSamples;
        public float GyroXRadS;
        public float GyroYRadS;
        public float GyroZRadS;
        public float AccXG;
        public float AccYG;
        public float AccZG;
        public float AvgGyroXRadS;
        public float AvgGyroYRadS;
        public float AvgGyroZRadS;
        public float AvgAccXG;
        public float AvgAccYG;
        public float AvgAccZG;
        public float AccelStdG;
        public float GyroRmsRadS;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    internal struct NativeFrameInfo
    {
        public ulong TimestampUs;
        public uint PointCount;
        public uint DroppedPoints;
        public uint PacketCount;
        public uint LidarHandle;
    }

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl,
        CharSet = CharSet.Ansi)]
    internal static extern int LivoxHmi_Initialize(string configPath);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void LivoxHmi_Shutdown();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int LivoxHmi_IsInitialized();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern uint LivoxHmi_GetQueueDepth();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern ulong LivoxHmi_GetPacketsReceived();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern ulong LivoxHmi_GetPacketsDropped();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern uint LivoxHmi_GetApiVersion();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int LivoxHmi_GetImuSnapshot(out NativeImuSnapshot snapshot);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int LivoxHmi_RequestHms();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern uint LivoxHmi_GetHmsCodes([Out] uint[] outCodes, uint capacity);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int LivoxHmi_GetHmsQueryStatus();

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int LivoxHmi_TryGetFrame(
        IntPtr outPoints, uint capacity, out NativeFrameInfo outInfo);
}

public readonly record struct LivoxImuSnapshot(
    DateTimeOffset Timestamp,
    ulong TotalSamples,
    uint LidarHandle,
    uint WindowSamples,
    float GyroXRadS, float GyroYRadS, float GyroZRadS,
    float AccXG, float AccYG, float AccZG,
    float AvgGyroXRadS, float AvgGyroYRadS, float AvgGyroZRadS,
    float AvgAccXG, float AvgAccYG, float AvgAccZG,
    float AccelStdG, float GyroRmsRadS)
{
    public double AccelMagnitudeG => Math.Sqrt(AvgAccXG * AvgAccXG + AvgAccYG * AvgAccYG + AvgAccZG * AvgAccZG);
    public double GyroRmsDegS => GyroRmsRadS * 180.0 / Math.PI;
    // Native v1602 reports gyro residual RMS (noise around the window mean), not absolute gyro magnitude.
    // This avoids a constant gyro bias preventing STABLE forever on a stationary sensor.
    public bool HasEnoughSamples => WindowSamples >= 40;
    public bool AccelMagnitudeOk => AccelMagnitudeG is > 0.85 and < 1.15;
    public bool AccelNoiseOk => AccelStdG < 0.060f;
    public bool GyroNoiseOk => GyroRmsRadS < 0.035f;
    public bool IsStable => HasEnoughSamples && AccelMagnitudeOk && AccelNoiseOk && GyroNoiseOk;
    public string StabilityReason
    {
        get
        {
            if (!HasEnoughSamples) return $"collecting {WindowSamples}/40";
            if (!AccelMagnitudeOk) return $"|a|={AccelMagnitudeG:0.000}g";
            if (!AccelNoiseOk) return $"acc noise={AccelStdG:0.0000}g";
            if (!GyroNoiseOk) return $"gyro noise={GyroRmsDegS:0.000}deg/s";
            return "stable";
        }
    }
}

public sealed class LivoxDeviceService : IPointCloudSource
{
    public const int MaxPoints = 120_000;
    public string Name => "Livox MID-360";
    public ulong FramesProduced { get; private set; }
    private readonly byte[] _raw = new byte[MaxPoints * 16];
    private GCHandle _pin;
    private bool _connected;
    private readonly object _nativeGate = new();

    public bool IsConnected => _connected;
    public ulong PacketsReceived => LivoxNative.LivoxHmi_GetPacketsReceived();
    public ulong PacketsDropped => LivoxNative.LivoxHmi_GetPacketsDropped();
    public uint QueueDepth => LivoxNative.LivoxHmi_GetQueueDepth();
    public uint NativeApiVersion => LivoxNative.LivoxHmi_GetApiVersion();

    public bool TryGetImuSnapshot(out LivoxImuSnapshot snapshot)
    {
        lock (_nativeGate)
        {
            snapshot = default;
            if (!_connected) return false;
            var rc = LivoxNative.LivoxHmi_GetImuSnapshot(out var s);
            if (rc <= 0) return false;
            snapshot = new LivoxImuSnapshot(
                DateTimeOffset.UtcNow, s.TotalSamples, s.LidarHandle, s.WindowSamples,
                s.GyroXRadS, s.GyroYRadS, s.GyroZRadS,
                s.AccXG, s.AccYG, s.AccZG,
                s.AvgGyroXRadS, s.AvgGyroYRadS, s.AvgGyroZRadS,
                s.AvgAccXG, s.AvgAccYG, s.AvgAccZG,
                s.AccelStdG, s.GyroRmsRadS);
            return true;
        }
    }

    public int RequestHms()
    {
        lock (_nativeGate)
            return _connected ? LivoxNative.LivoxHmi_RequestHms() : -2;
    }

    public IReadOnlyList<uint> GetHmsCodes()
    {
        lock (_nativeGate)
        {
            if (!_connected) return Array.Empty<uint>();
            var raw = new uint[8];
            var count = (int)Math.Min((uint)raw.Length, LivoxNative.LivoxHmi_GetHmsCodes(raw, (uint)raw.Length));
            if (count <= 0) return Array.Empty<uint>();
            return raw.Take(count).Where(v => v != 0).ToArray();
        }
    }

    public int HmsQueryStatus
    {
        get { lock (_nativeGate) return _connected ? LivoxNative.LivoxHmi_GetHmsQueryStatus() : 0; }
    }

    public void Connect(string configPath)
    {
        lock (_nativeGate)
        {
            if (_connected) return;
            var fullPath = Path.GetFullPath(configPath);
            if (!File.Exists(fullPath))
                throw new FileNotFoundException("Livox config not found.", fullPath);

            try
            {
                var api = LivoxNative.LivoxHmi_GetApiVersion();
                if (api < 1601)
                    throw new InvalidOperationException($"LivoxHmiBridge.dll is outdated (API {api}). Rebuild native bridge for v1.7.");

                if (LivoxNative.LivoxHmi_Initialize(fullPath) != 1)
                    throw new InvalidOperationException("Livox SDK initialization failed.");

                _pin = GCHandle.Alloc(_raw, GCHandleType.Pinned);
                _connected = true;
            }
            catch
            {
                if (_pin.IsAllocated) _pin.Free();
                try { LivoxNative.LivoxHmi_Shutdown(); } catch { }
                _connected = false;
                throw;
            }
        }
    }

    public void Disconnect()
    {
        lock (_nativeGate)
        {
            if (!_connected) return;
            try { LivoxNative.LivoxHmi_Shutdown(); }
            finally
            {
                if (_pin.IsAllocated) _pin.Free();
                _connected = false;
            }
        }
    }

    public bool TryGetFrame(out PointCloudFrame? frame)
    {
        frame = null;
        lock (_nativeGate)
        {
            if (!_connected) return false;

            var rc = LivoxNative.LivoxHmi_TryGetFrame(
                _pin.AddrOfPinnedObject(), MaxPoints, out var info);

            if (rc <= 0) return false;

            var count = Math.Min((int)info.PointCount, MaxPoints);
            var points = new Point3D[count];
            var span = MemoryMarshal.Cast<byte, LivoxNative.NativePoint>(_raw.AsSpan());
            for (int i = 0; i < count; i++)
            {
                var p = span[i];
                points[i] = new Point3D(p.X, p.Y, p.Z, p.Reflectivity, p.Tag);
            }

            FramesProduced++;

            frame = new PointCloudFrame(
                DateTimeOffset.UtcNow,
                info.LidarHandle,
                points,
                checked((int)info.DroppedPoints),
                checked((int)info.PacketCount));

            return true;
        }
    }

    public void Dispose() => Disconnect();
}
