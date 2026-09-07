namespace LivoxHmi.Core;

public enum LivoxTagConfidence : byte
{
    HighConfidenceNormal = 0,
    Moderate = 1,
    Low = 2,
    Reserved = 3
}

public readonly record struct LivoxTagDecoded(
    LivoxTagConfidence Other,
    LivoxTagConfidence Atmospheric,
    LivoxTagConfidence Dragging)
{
    public bool IsNormal => Other == LivoxTagConfidence.HighConfidenceNormal &&
                            Atmospheric == LivoxTagConfidence.HighConfidenceNormal &&
                            Dragging == LivoxTagConfidence.HighConfidenceNormal;
    public bool IsAtmosphericCandidate => Atmospheric is LivoxTagConfidence.Moderate or LivoxTagConfidence.Low;
}

public static class LivoxTagDecoder
{
    // Mid-360 tag byte (Livox protocol):
    // bits 7-6 reserved; 5-4 other noise; 3-2 rain/fog/dust/tiny particles;
    // 1-0 dragging/glue noise. Value 0/1/2/3 = high/moderate/low/reserved.
    public static LivoxTagDecoded Decode(byte tag) => new(
        (LivoxTagConfidence)((tag >> 4) & 0x03),
        (LivoxTagConfidence)((tag >> 2) & 0x03),
        (LivoxTagConfidence)(tag & 0x03));
}

public enum HmsSeverity : byte
{
    None = 0,
    Info = 1,
    Warning = 2,
    Error = 3,
    Fatal = 4
}

public sealed record HmsDiagnostic(
    uint RawCode,
    ushort AbnormalId,
    HmsSeverity Severity,
    string Description,
    string SuggestedAction);

public static class Mid360HmsDecoder
{
    public static HmsDiagnostic Decode(uint raw)
    {
        var id = (ushort)(raw >> 16);
        var severity = (HmsSeverity)(raw & 0xFF);
        var (description, action) = Lookup(id, severity);
        return new HmsDiagnostic(raw, id, severity, description, action);
    }

    private static (string Description, string Action) Lookup(ushort id, HmsSeverity severity)
    {
        if (id >= 0x0210 && id <= 0x0219)
            return severity == HmsSeverity.Fatal
                ? ("Scan module is abnormal", "Restart the device to restore operation")
                : ("Scan module is abnormal; automatic recovery is in progress", "Wait; if it persists, restart the device");

        return id switch
        {
            0x0102 => ("Environment temperature is slightly high", "Check ambient temperature and heat dissipation"),
            0x0103 => ("Environment temperature is relatively high", "Check ambient temperature and heat dissipation"),
            0x0104 => ("LiDAR window is dirty; point-cloud reliability may be affected", "Clean the LiDAR window"),
            0x0105 => ("Device upgrade process error", "Restart the upgrade process"),
            0x0111 => ("Abnormal internal component temperature", "Check ambient temperature and heat dissipation"),
            0x0112 => ("Abnormal internal component temperature", "Check ambient temperature and heat dissipation"),
            0x0113 => ("IMU stopped working", "Restart the device"),
            0x0114 => ("Environment temperature is high", "Check ambient temperature and heat dissipation"),
            0x0115 => ("Environment temperature exceeded the limit; device stopped", "Check ambient temperature and heat dissipation"),
            0x0116 => ("Abnormal external voltage", "Check the external power supply voltage"),
            0x0117 => ("Abnormal LiDAR parameters", "Restart the device"),
            0x0118 => ("Internal device components are damaged", "Contact maintenance/service personnel"),
            0x0201 => ("Scan module is heating", "Wait for scan-module warm-up"),
            0x0401 => ("Communication link recovered after link-down", "Check the communication link"),
            0x0402 => ("PTP synchronization stopped or time gap is too large", "Check the PTP time source"),
            0x0403 => ("Unsupported PTP IEEE 1588-v2.1 detected", "Use IEEE 1588-v2.0 instead"),
            0x0404 => ("PPS time synchronization abnormal", "Check PPS and GPS signals"),
            0x0405 => ("Time synchronization exception", "Check the synchronization source and topology"),
            0x0406 => ("Time synchronization accuracy is low", "Check the time source"),
            0x0407 => ("PPS synchronization failed due to GPS signal loss", "Check the GPS signal"),
            0x0408 => ("PPS synchronization failed due to PPS signal loss", "Check the PPS signal"),
            0x0409 => ("GPS signal is abnormal", "Check the GPS time source"),
            0x040A => ("PTP and gPTP signals exist at the same time", "Use only PTP or only gPTP on the network"),
            _ => ($"Unknown MID-360 HMS abnormal ID 0x{id:X4}", "Refer to the current Livox MID-360 HMS diagnostic table")
        };
    }
}
