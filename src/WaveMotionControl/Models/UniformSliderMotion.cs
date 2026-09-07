namespace WaveMotionControl.Models;

/// <summary>
/// Vị trí cảm biến Home của cơ cấu tay quay - con trượt.
/// </summary>
public enum SliderHomePoint
{
    OuterDeadCenter,
    InnerDeadCenter
}

/// <summary>
/// Chiều quay liên tục của motor trong bài test 16 PR.
/// </summary>
public enum UniformSliderMotorDirection
{
    Forward,
    Reverse
}

/// <summary>
/// Thông số hình học và tốc độ dùng để tạo bảng 16 PR.
/// </summary>
public sealed record UniformSliderMotionSettings(
    double CrankRadiusMm,
    double ConnectingRodLengthMm,
    double OffsetMm,
    double SliderSpeedMmPerSecond,
    double PeakCurrentAmps,
    int AccelerationMsPer1000Rpm,
    int DecelerationMsPer1000Rpm,
    SliderHomePoint HomePoint,
    UniformSliderMotorDirection MotorDirection)
{
    public static UniformSliderMotionSettings Default { get; } = new(
        CrankRadiusMm: 50.0,
        ConnectingRodLengthMm: 100.0,
        OffsetMm: 15.0,
        SliderSpeedMmPerSecond: 20.0,
        PeakCurrentAmps: 3.0,
        AccelerationMsPer1000Rpm: 1000,
        DecelerationMsPer1000Rpm: 1000,
        HomePoint: SliderHomePoint.OuterDeadCenter,
        MotorDirection: UniformSliderMotorDirection.Forward);
}

/// <summary>
/// Một đoạn PR tương đối. Driver chạy xong đoạn này sẽ Jump sang PR kế tiếp.
/// </summary>
public sealed record UniformSliderPrSegment(
    int PathIndex,
    int RelativePulses,
    int SpeedRpm,
    int AccelerationMsPer1000Rpm,
    int DecelerationMsPer1000Rpm,
    double SliderStartMm,
    double SliderEndMm);

/// <summary>
/// Kết quả tính bảng 16 PR cho một vòng quay tay quay.
/// </summary>
public sealed record UniformSliderMotionPlan(
    int PulsesPerRevolution,
    double StrokeMm,
    double OuterDeadCenterAngleDeg,
    double InnerDeadCenterAngleDeg,
    double DesiredCycleTimeSeconds,
    int MinimumSpeedRpm,
    int MaximumSpeedRpm,
    IReadOnlyList<UniformSliderPrSegment> Segments);

/// <summary>
/// Tạo 16 PR: 8 đoạn cho một chiều con trượt và 8 đoạn cho chiều còn lại.
/// Các đoạn được chia đều theo quãng đường con trượt, không chia đều theo góc tay quay.
/// </summary>
public static class UniformSliderMotionPlanner
{
    public const int PathCount = 16;
    public const int SegmentsPerStroke = PathCount / 2;

    public static UniformSliderMotionPlan Build(
        UniformSliderMotionSettings settings,
        int pulsesPerRevolution)
    {
        ArgumentNullException.ThrowIfNull(settings);

        if (!double.IsFinite(settings.CrankRadiusMm) ||
            settings.CrankRadiusMm <= 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(settings),
                "Bán kính tay quay phải lớn hơn 0.");
        }

        if (!double.IsFinite(settings.ConnectingRodLengthMm) ||
            settings.ConnectingRodLengthMm <= settings.CrankRadiusMm)
        {
            throw new ArgumentOutOfRangeException(
                nameof(settings),
                "Thanh truyền phải dài hơn tay quay.");
        }

        if (!double.IsFinite(settings.OffsetMm) || settings.OffsetMm < 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(settings),
                "Độ lệch tâm phải lớn hơn hoặc bằng 0.");
        }

        var fullRotationLimit =
            settings.ConnectingRodLengthMm - settings.CrankRadiusMm;

        if (settings.OffsetMm >= fullRotationLimit)
        {
            throw new ArgumentOutOfRangeException(
                nameof(settings),
                $"Để tay quay quay đủ 360°, độ lệch tâm phải nhỏ hơn " +
                $"L-R = {fullRotationLimit:0.###} mm.");
        }

        if (!double.IsFinite(settings.SliderSpeedMmPerSecond) ||
            settings.SliderSpeedMmPerSecond <= 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(settings),
                "Tốc độ con trượt phải lớn hơn 0.");
        }

        if (!double.IsFinite(settings.PeakCurrentAmps) ||
            settings.PeakCurrentAmps is < 0.5 or > 6.0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(settings),
                "Dòng test 16 PR phải từ 0,5 A đến 6,0 A.");
        }

        if (pulsesPerRevolution is < 200 or > 51_200)
        {
            throw new ArgumentOutOfRangeException(
                nameof(pulsesPerRevolution),
                "Pulse/vòng phải từ 200 đến 51.200.");
        }

        var radius = settings.CrankRadiusMm;
        var rod = settings.ConnectingRodLengthMm;
        var offset = settings.OffsetMm;

        var xOuter = Math.Sqrt(
            (rod + radius) * (rod + radius) - offset * offset);
        var xInner = Math.Sqrt(
            (rod - radius) * (rod - radius) - offset * offset);
        var stroke = xOuter - xInner;

        var thetaOuter = Math.Atan2(offset, xOuter);
        var thetaInner = Math.Atan2(offset, xInner) + Math.PI;

        var angles = new List<double>(PathCount + 1);
        var sliderPositions = new List<double>(PathCount + 1);

        // Đầu ngoài -> đầu trong: nhánh theta = phi + beta.
        for (var index = 0; index <= SegmentsPerStroke; index++)
        {
            var x = xOuter - stroke * index / SegmentsPerStroke;
            sliderPositions.Add(x);
            angles.Add(GetCrankAngleForForwardStroke(
                x,
                radius,
                rod,
                offset));
        }

        // Đầu trong -> đầu ngoài: nhánh theta = phi - beta + 2*pi.
        // Bỏ điểm đầu trong bị trùng.
        for (var index = 1; index <= SegmentsPerStroke; index++)
        {
            var x = xInner + stroke * index / SegmentsPerStroke;
            sliderPositions.Add(x);
            angles.Add(GetCrankAngleForReturnStroke(
                x,
                radius,
                rod,
                offset));
        }

        var cumulativePulses = angles
            .Select(angle => (int)Math.Round(
                (angle - thetaOuter) /
                (2.0 * Math.PI) *
                pulsesPerRevolution))
            .ToArray();

        // Khóa chính xác tổng một vòng, tránh sai số làm tròn tích lũy.
        cumulativePulses[0] = 0;
        cumulativePulses[^1] = pulsesPerRevolution;

        var sliderDistancePerSegment = stroke / SegmentsPerStroke;
        var segmentTimeSeconds =
            sliderDistancePerSegment /
            settings.SliderSpeedMmPerSecond;

        var positiveCycle = new List<UniformSliderPrSegment>(PathCount);

        for (var index = 0; index < PathCount; index++)
        {
            var pulseDelta =
                cumulativePulses[index + 1] -
                cumulativePulses[index];

            if (pulseDelta <= 0)
            {
                throw new InvalidOperationException(
                    $"PR{index} có số pulse không hợp lệ ({pulseDelta}). " +
                    "Hãy tăng Pulse/vòng.");
            }

            var exactRpm =
                pulseDelta /
                (double)pulsesPerRevolution *
                60.0 /
                segmentTimeSeconds;

            if (exactRpm > 5000.0)
            {
                throw new InvalidOperationException(
                    $"Tốc độ yêu cầu của PR{index} là {exactRpm:0.0} rpm, " +
                    "vượt giới hạn 5000 rpm. Hãy giảm tốc độ con trượt.");
            }

            var speedRpm = Math.Clamp(
                (int)Math.Round(exactRpm),
                1,
                5000);

            positiveCycle.Add(new UniformSliderPrSegment(
                PathIndex: index,
                RelativePulses: pulseDelta,
                SpeedRpm: speedRpm,
                AccelerationMsPer1000Rpm: Math.Clamp(
                    settings.AccelerationMsPer1000Rpm,
                    1,
                    10_000),
                DecelerationMsPer1000Rpm: Math.Clamp(
                    settings.DecelerationMsPer1000Rpm,
                    1,
                    10_000),
                SliderStartMm: sliderPositions[index],
                SliderEndMm: sliderPositions[index + 1]));
        }

        var ordered = new List<UniformSliderPrSegment>(PathCount);

        if (settings.MotorDirection == UniformSliderMotorDirection.Forward)
        {
            var startBaseIndex =
                settings.HomePoint == SliderHomePoint.OuterDeadCenter
                    ? 0
                    : SegmentsPerStroke;

            for (var pathIndex = 0; pathIndex < PathCount; pathIndex++)
            {
                var source =
                    positiveCycle[(startBaseIndex + pathIndex) % PathCount];

                ordered.Add(source with { PathIndex = pathIndex });
            }
        }
        else
        {
            var startBaseIndex =
                settings.HomePoint == SliderHomePoint.OuterDeadCenter
                    ? PathCount - 1
                    : SegmentsPerStroke - 1;

            for (var pathIndex = 0; pathIndex < PathCount; pathIndex++)
            {
                var source =
                    positiveCycle[
                        (startBaseIndex - pathIndex + PathCount) % PathCount];

                ordered.Add(new UniformSliderPrSegment(
                    PathIndex: pathIndex,
                    RelativePulses: -source.RelativePulses,
                    SpeedRpm: source.SpeedRpm,
                    AccelerationMsPer1000Rpm:
                        source.AccelerationMsPer1000Rpm,
                    DecelerationMsPer1000Rpm:
                        source.DecelerationMsPer1000Rpm,
                    SliderStartMm: source.SliderEndMm,
                    SliderEndMm: source.SliderStartMm));
            }
        }

        var signedPulseSum = ordered.Sum(segment => segment.RelativePulses);
        var expectedPulseSum =
            settings.MotorDirection == UniformSliderMotorDirection.Forward
                ? pulsesPerRevolution
                : -pulsesPerRevolution;

        if (signedPulseSum != expectedPulseSum)
        {
            throw new InvalidOperationException(
                $"Tổng pulse của chu kỳ là {signedPulseSum}, " +
                $"không bằng {expectedPulseSum}.");
        }

        return new UniformSliderMotionPlan(
            PulsesPerRevolution: pulsesPerRevolution,
            StrokeMm: stroke,
            OuterDeadCenterAngleDeg: thetaOuter * 180.0 / Math.PI,
            InnerDeadCenterAngleDeg: thetaInner * 180.0 / Math.PI,
            DesiredCycleTimeSeconds:
                2.0 * stroke / settings.SliderSpeedMmPerSecond,
            MinimumSpeedRpm: ordered.Min(segment => segment.SpeedRpm),
            MaximumSpeedRpm: ordered.Max(segment => segment.SpeedRpm),
            Segments: ordered);
    }

    private static double GetCrankAngleForForwardStroke(
        double sliderX,
        double radius,
        double rod,
        double offset)
    {
        var distance = Math.Sqrt(
            sliderX * sliderX + offset * offset);
        var phi = Math.Atan2(offset, sliderX);
        var cosBeta =
            (radius * radius + distance * distance - rod * rod) /
            (2.0 * radius * distance);
        var beta = Math.Acos(Math.Clamp(cosBeta, -1.0, 1.0));

        return phi + beta;
    }

    private static double GetCrankAngleForReturnStroke(
        double sliderX,
        double radius,
        double rod,
        double offset)
    {
        var distance = Math.Sqrt(
            sliderX * sliderX + offset * offset);
        var phi = Math.Atan2(offset, sliderX);
        var cosBeta =
            (radius * radius + distance * distance - rod * rod) /
            (2.0 * radius * distance);
        var beta = Math.Acos(Math.Clamp(cosBeta, -1.0, 1.0));

        return phi - beta + 2.0 * Math.PI;
    }
}
