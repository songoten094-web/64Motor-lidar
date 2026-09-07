namespace WaveMotionControl.Models;

/// <summary>
/// AUTO: bản đồ 16x16, nhiều cụm độc lập và các hiệu ứng theo cụm.
/// 1 vòng motor = 1 chu kỳ đi + về của con trượt.
/// </summary>
public enum AutoEffectType
{
    WaveFromCenter,
    WaveHeadToTail,
    Lidar
}

public enum AutoWaveDirection
{
    LeftToRight,
    RightToLeft,
    TopToBottom,
    BottomToTop
}

public sealed record AutoGridCell(
    int Row,
    int Column,
    AxisAddress? DriverId);

public sealed record AutoWaveLayer(
    int Index,
    double DistanceSquared,
    IReadOnlyList<AxisAddress> Drivers);

public sealed record AutoCluster(
    int Id,
    int TopRow,
    int LeftColumn,
    int Width,
    int Height,
    IReadOnlyList<AutoGridCell> Cells,
    AutoEffectType Effect,
    AutoWaveDirection WaveDirection,
    double LayerOffsetRevolutions,
    double FrequencyHz,
    int LidarRandomSeed = 0)
{
    public const double LidarCenterTargetRevolutions = 0.500;
    public const double LidarFalloffPerColumnRevolutions = 0.125;

    public double CenterRow => TopRow + (Height - 1) / 2.0;
    public double CenterColumn => LeftColumn + (Width - 1) / 2.0;

    public IReadOnlyList<AutoWaveLayer> BuildWaveLayers()
    {
        return Effect switch
        {
            AutoEffectType.WaveHeadToTail => BuildHeadToTailLayers(),
            AutoEffectType.Lidar => BuildLidarColumnLayers(),
            _ => BuildCenterLayers()
        };
    }

    /// <summary>
    /// LIDAR: 1 Zone tương ứng 1 cột của cụm.
    /// Zone/column dùng zero-based ở tầng model.
    /// </summary>
    public double GetLidarTargetRevolutions(int activeZoneColumn, int localColumn)
    {
        var safeZone = Math.Clamp(activeZoneColumn, 0, Math.Max(0, Width - 1));
        var safeColumn = Math.Clamp(localColumn, 0, Math.Max(0, Width - 1));
        var distance = Math.Abs(safeColumn - safeZone);
        return Math.Max(
            0,
            LidarCenterTargetRevolutions -
            distance * LidarFalloffPerColumnRevolutions);
    }

    /// <summary>
    /// Pha nền RANDOM của LIDAR. Giá trị ổn định theo seed + ô + Driver ID,
    /// để toàn bộ motor cùng tốc độ nhưng có pha khởi đầu khác nhau.
    /// </summary>
    public double GetLidarRandomPhase(AxisAddress driver)
    {
        var cell = Cells.FirstOrDefault(c => c.DriverId == driver);
        if (cell is null)
        {
            return 0;
        }

        unchecked
        {
            uint x = (uint)(LidarRandomSeed == 0 ? 0x51F15EED : LidarRandomSeed);
            x ^= (uint)(cell.Row + 1) * 0x9E3779B9u;
            x = (x << 13) | (x >> 19);
            x ^= (uint)(cell.Column + 1) * 0x85EBCA6Bu;
            x = (x << 11) | (x >> 21);
            x ^= (uint)(driver.Line * 37 + driver.SlaveId * 101) * 0xC2B2AE35u;
            x ^= x >> 16;
            x *= 0x7FEB352Du;
            x ^= x >> 15;

            // Tránh đúng 0.000 để khi START nhìn thấy rõ các pha khác nhau.
            return 0.02 + (x % 9600u) / 10000.0;
        }
    }

    public int GetLocalColumn(AxisAddress driver)
    {
        var cell = Cells.FirstOrDefault(c => c.DriverId == driver);
        return cell is null ? 0 : Math.Clamp(cell.Column - LeftColumn, 0, Math.Max(0, Width - 1));
    }

    private IReadOnlyList<AutoWaveLayer> BuildCenterLayers()
    {
        // Layer dạng vòng chữ nhật đồng tâm (Chebyshev rings):
        // 5x5 => 2 2 2 2 2 / 2 1 1 1 2 / 2 1 0 1 2 / ...
        // 4x4 => 1 1 1 1 / 1 0 0 1 / 1 0 0 1 / 1 1 1 1.
        // Dùng tọa độ nhân 2 để xử lý đồng nhất cụm chẵn/lẻ mà không cần số thực.
        return Cells
            .Where(c => c.DriverId is not null)
            .Select(c =>
            {
                var localRow2 = 2 * (c.Row - TopRow) - (Height - 1);
                var localCol2 = 2 * (c.Column - LeftColumn) - (Width - 1);
                var layerIndex = Math.Max(Math.Abs(localRow2), Math.Abs(localCol2)) / 2;
                return new { Cell = c, LayerIndex = layerIndex };
            })
            .GroupBy(x => x.LayerIndex)
            .OrderBy(g => g.Key)
            .Select(g => new AutoWaveLayer(
                g.Key,
                g.Key,
                g.Select(x => x.Cell.DriverId!.Value).Distinct().ToArray()))
            .ToArray();
    }

    private IReadOnlyList<AutoWaveLayer> BuildHeadToTailLayers()
    {
        // Phương án A: cả một hàng hoặc một cột là một layer và chạy đồng thời.
        return Cells
            .Where(c => c.DriverId is not null)
            .Select(c =>
            {
                var layerIndex = WaveDirection switch
                {
                    AutoWaveDirection.RightToLeft =>
                        (LeftColumn + Width - 1) - c.Column,
                    AutoWaveDirection.TopToBottom =>
                        c.Row - TopRow,
                    AutoWaveDirection.BottomToTop =>
                        (TopRow + Height - 1) - c.Row,
                    _ =>
                        c.Column - LeftColumn
                };

                return new { Cell = c, LayerIndex = layerIndex };
            })
            .GroupBy(x => x.LayerIndex)
            .OrderBy(g => g.Key)
            .Select(g => new AutoWaveLayer(
                g.Key,
                g.Key,
                g.Select(x => x.Cell.DriverId!.Value).Distinct().ToArray()))
            .ToArray();
    }

    private IReadOnlyList<AutoWaveLayer> BuildLidarColumnLayers()
    {
        // LIDAR: mỗi cột chính là một Zone/layer logic.
        return Cells
            .Where(c => c.DriverId is not null)
            .GroupBy(c => c.Column - LeftColumn)
            .OrderBy(g => g.Key)
            .Select(g => new AutoWaveLayer(
                g.Key,
                g.Key,
                g.Select(c => c.DriverId!.Value).Distinct().ToArray()))
            .ToArray();
    }
}

public sealed record AutoProgram(
    int GridRows,
    int GridColumns,
    IReadOnlyList<AutoGridCell> GridCells,
    IReadOnlyList<AutoCluster> Clusters,
    int PulsesPerRevolution,
    double FrequencyHz,
    double LayerOffsetRevolutions,
    double RampUpSeconds,
    double RampDownSeconds)
{
    public const int GridSize = 16;

    public IReadOnlyDictionary<AxisAddress, AutoCluster> DriverClusters()
    {
        var result = new Dictionary<AxisAddress, AutoCluster>();
        foreach (var cluster in Clusters)
        {
            foreach (var cell in cluster.Cells)
            {
                if (cell.DriverId is AxisAddress driver)
                {
                    result[driver] = cluster;
                }
            }
        }

        return result;
    }
}
