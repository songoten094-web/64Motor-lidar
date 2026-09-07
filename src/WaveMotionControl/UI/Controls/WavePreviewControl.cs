using System.ComponentModel;
using System.Drawing.Drawing2D;
using WaveMotionControl.Models;

namespace WaveMotionControl.UI.Controls;

[DesignerCategory("Component")]
public class WavePreviewControl : Control
{
    private readonly System.Windows.Forms.Timer _timer;
    private double _timeSeconds;
    private DateTime _lastTick = DateTime.UtcNow;

    public WavePreviewControl()
    {
        DoubleBuffered = true;
        BackColor = UiTheme.Surface;
        ForeColor = UiTheme.Text;
        MinimumSize = new Size(500, 420);
        _timer = new System.Windows.Forms.Timer { Interval = 40 };
        _timer.Tick += (_, _) =>
        {
            var now = DateTime.UtcNow;
            if (Running && !Paused)
            {
                _timeSeconds += Math.Max(0, (now - _lastTick).TotalSeconds);
            }
            _lastTick = now;
            Invalidate();
        };
        _timer.Start();
    }

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public Func<AutoProgram?>? ProgramProvider { get; set; }

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public Func<int, int?>? LidarZoneProvider { get; set; }

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public bool Running { get; set; }

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public bool Paused { get; set; }

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public AxisAddress InspectAxis { get; set; } = new(1, 1);

    [Browsable(false)]
    public double InspectPositionRevolutions { get; private set; }

    [Browsable(false)]
    public double InspectPhaseDegrees { get; private set; }

    [Browsable(false)]
    public double CurrentTimeSeconds => _timeSeconds;

    public void ResetTime()
    {
        _timeSeconds = 0;
        _lastTick = DateTime.UtcNow;
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
        e.Graphics.Clear(BackColor);

        var program = ProgramProvider?.Invoke();
        if (program is null || program.Clusters.Count == 0)
        {
            DrawCenteredMessage(e.Graphics, "Chưa tạo cụm AUTO");
            return;
        }

        var margin = 20;
        var area = new RectangleF(margin, margin, Math.Max(10, ClientSize.Width - 2 * margin), Math.Max(10, ClientSize.Height - 2 * margin));
        var cellSize = Math.Min(area.Width / 16f, area.Height / 16f);
        var gridW = cellSize * 16;
        var gridH = cellSize * 16;
        var origin = new PointF(area.Left + (area.Width - gridW) / 2f, area.Top + (area.Height - gridH) / 2f);

        using var gridPen = new Pen(Color.FromArgb(55, 70, 90), 1);
        using var clusterPen = new Pen(Color.FromArgb(70, 170, 255), 2);
        using var font = new Font("Segoe UI", Math.Max(6, cellSize * 0.23f), FontStyle.Bold);
        using var smallFont = new Font("Segoe UI", Math.Max(6, cellSize * 0.18f));

        for (var r = 0; r < 16; r++)
        {
            for (var c = 0; c < 16; c++)
            {
                var rect = new RectangleF(origin.X + c * cellSize, origin.Y + r * cellSize, cellSize, cellSize);
                e.Graphics.DrawRectangle(gridPen, rect.X, rect.Y, rect.Width, rect.Height);
            }
        }

        foreach (var cluster in program.Clusters)
        {
            var layers = cluster.BuildWaveLayers();
            var layerMap = new Dictionary<AxisAddress, int>();
            foreach (var layer in layers)
            {
                foreach (var driver in layer.Drivers) layerMap[driver] = layer.Index;
            }

            foreach (var cell in cluster.Cells)
            {
                var rect = new RectangleF(origin.X + cell.Column * cellSize + 1, origin.Y + cell.Row * cellSize + 1, cellSize - 2, cellSize - 2);
                if (cell.DriverId is AxisAddress driver)
                {
                    var layer = layerMap.TryGetValue(driver, out var li) ? li : 0;
                    var maxLayerIndex = layers.Count == 0 ? 0 : layers.Max(x => x.Index);
                    var phase = GetDriverPhase(cluster, driver, layer, maxLayerIndex, _timeSeconds);
                    var normalized = phase >= 0 ? phase % 1.0 : 0;
                    var color = ColorFromPhase(normalized, layer);
                    using var brush = new SolidBrush(Color.FromArgb(190, color));
                    e.Graphics.FillRectangle(brush, rect);
                    var text = driver.DisplayId;
                    var size = e.Graphics.MeasureString(text, font);
                    e.Graphics.DrawString(text, font, Brushes.White, rect.X + (rect.Width - size.Width) / 2, rect.Y + (rect.Height - size.Height) / 2);
                }
                else
                {
                    using var brush = new SolidBrush(Color.FromArgb(45, 50, 60));
                    e.Graphics.FillRectangle(brush, rect);
                }
            }

            var outline = new RectangleF(origin.X + cluster.LeftColumn * cellSize, origin.Y + cluster.TopRow * cellSize, cluster.Width * cellSize, cluster.Height * cellSize);
            e.Graphics.DrawRectangle(clusterPen, outline.X, outline.Y, outline.Width, outline.Height);
            e.Graphics.DrawString($"Cụm {cluster.Id}", smallFont, Brushes.White, outline.X + 3, Math.Max(0, outline.Y - 18));
        }

        UpdateInspection(program);
    }

    private void UpdateInspection(AutoProgram program)
    {
        var cluster = program.Clusters.FirstOrDefault(c => c.Cells.Any(x => x.DriverId == InspectAxis));
        if (cluster is null)
        {
            InspectPositionRevolutions = 0;
            InspectPhaseDegrees = 0;
            return;
        }

        var layer = cluster.BuildWaveLayers().FirstOrDefault(l => l.Drivers.Contains(InspectAxis));
        if (layer is null)
        {
            InspectPositionRevolutions = 0;
            InspectPhaseDegrees = 0;
            return;
        }

        var layers = cluster.BuildWaveLayers();
        var maxLayerIndex = layers.Count == 0 ? 0 : layers.Max(x => x.Index);
        var phase = GetDriverPhase(cluster, InspectAxis, layer.Index, maxLayerIndex, _timeSeconds);
        InspectPositionRevolutions = Math.Max(0, phase);
        InspectPhaseDegrees = ((phase * 360.0) % 360.0 + 360.0) % 360.0;
    }

    private double GetDriverPhase(
        AutoCluster cluster,
        AxisAddress driver,
        int layerIndex,
        int maxLayerIndex,
        double timeSeconds)
    {
        if (cluster.Effect == AutoEffectType.Lidar)
        {
            var activeZone = LidarZoneProvider?.Invoke(cluster.Id);
            if (activeZone is int zone)
            {
                var localColumn = cluster.GetLocalColumn(driver);
                return cluster.GetLidarTargetRevolutions(zone, localColumn) +
                       timeSeconds * Math.Max(0.0001, cluster.FrequencyHz);
            }

            return cluster.GetLidarRandomPhase(driver) +
                   timeSeconds * Math.Max(0.0001, cluster.FrequencyHz);
        }

        return GetLayerPhase(cluster, layerIndex, maxLayerIndex, timeSeconds);
    }

    private static double GetLayerPhase(
        AutoCluster cluster,
        int layerIndex,
        int maxLayerIndex,
        double timeSeconds)
    {
        var frequency = Math.Max(0.0001, cluster.FrequencyHz);
        var rawPhaseOffset =
            (maxLayerIndex - layerIndex) *
            Math.Max(0, cluster.LayerOffsetRevolutions);
        var phaseOffset = rawPhaseOffset % 1.0;
        return phaseOffset + timeSeconds * frequency;
    }

    private static Color ColorFromPhase(double phase, int layer)
    {
        var hue = (float)((phase * 360.0 + layer * 32.0) % 360.0);
        return ColorFromHsv(hue, 0.72f, 0.92f);
    }

    private static Color ColorFromHsv(float hue, float saturation, float value)
    {
        var h = hue / 60f;
        var c = value * saturation;
        var x = c * (1 - Math.Abs(h % 2 - 1));
        var m = value - c;
        (float r, float g, float b) = h switch
        {
            < 1 => (c, x, 0f),
            < 2 => (x, c, 0f),
            < 3 => (0f, c, x),
            < 4 => (0f, x, c),
            < 5 => (x, 0f, c),
            _ => (c, 0f, x)
        };
        return Color.FromArgb((int)((r + m) * 255), (int)((g + m) * 255), (int)((b + m) * 255));
    }

    private static void DrawCenteredMessage(Graphics g, string text)
    {
        using var font = new Font("Segoe UI", 12F, FontStyle.Bold);
        var size = g.MeasureString(text, font);
        using var brush = new SolidBrush(Color.FromArgb(130, 145, 165));
        g.DrawString(text, font, brush, (g.VisibleClipBounds.Width - size.Width) / 2, (g.VisibleClipBounds.Height - size.Height) / 2);
    }
}
