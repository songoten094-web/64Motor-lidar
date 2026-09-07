using System.ComponentModel;
using WaveMotionControl.Models;
using WaveMotionControl.State;
using WaveMotionControl.UI.Controls;

namespace WaveMotionControl.UI.Pages;

[DesignerCategory("UserControl")]
public partial class StatusPage : UserControl
{
    private readonly ApplicationState _state;
    private readonly MetricCard _onlineMetric = new("Driver Online");
    private readonly MetricCard _homedMetric = new("Đã lấy gốc");
    private readonly MetricCard _movingMetric = new("Đang chuyển động");
    private readonly MetricCard _alarmMetric = new("Trục báo lỗi (Alarm)");
    private readonly DataGridView _grid;
    private readonly System.Windows.Forms.Timer _updateTimer;

    public StatusPage() : this(new ApplicationState())
    {
    }

    public StatusPage(ApplicationState state)
    {
        _state = state;

        InitializeComponent();

        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 3,
            BackColor = UiTheme.Background
        };
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 90));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));

        // Top Metrics
        var metrics = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 4, RowCount = 1 };
        for (var i = 0; i < 4; i++) metrics.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25F));
        metrics.Controls.Add(_onlineMetric, 0, 0);
        metrics.Controls.Add(_homedMetric, 1, 0);
        metrics.Controls.Add(_movingMetric, 2, 0);
        metrics.Controls.Add(_alarmMetric, 3, 0);
        root.Controls.Add(metrics, 0, 0);

        // Center Grid Card
        var centerCard = UiTheme.Card();
        centerCard.Dock = DockStyle.Fill;
        centerCard.Margin = new Padding(0, 8, 0, 8);
        var centerLayout = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 2, ColumnCount = 1 };
        centerLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 46));
        centerLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        var header = UiTheme.Label("BẢNG GIÁM SÁT CHI TIẾT 64 DRIVER EM2RS", UiTheme.FontSection, UiTheme.Text);
        header.Dock = DockStyle.Fill;
        header.TextAlign = ContentAlignment.MiddleLeft;
        header.AutoSize = false;
        centerLayout.Controls.Add(header, 0, 0);

        _grid = BuildDataGridView();
        centerLayout.Controls.Add(_grid, 0, 1);
        centerCard.Controls.Add(centerLayout);
        root.Controls.Add(centerCard, 0, 1);

        // Footer Summary Note
        var footerNote = UiTheme.Label("Dữ liệu được cập nhật thời gian thực mỗi 200ms · 4 tuyến Modbus RTU (Line 1 → Line 4)", UiTheme.FontSmall, UiTheme.Muted);
        footerNote.Dock = DockStyle.Fill;
        footerNote.TextAlign = ContentAlignment.MiddleCenter;
        footerNote.AutoSize = false;
        root.Controls.Add(footerNote, 0, 2);

        Controls.Add(root);

        PopulateGridRows();
        _updateTimer = new System.Windows.Forms.Timer { Interval = 200 };
        _updateTimer.Tick += (_, _) => UpdateDashboard();
        _updateTimer.Start();
        UpdateDashboard();

        Disposed += (_, _) => _updateTimer.Dispose();
    }

    private DataGridView BuildDataGridView()
    {
        var grid = new DataGridView
        {
            Dock = DockStyle.Fill,
            BackgroundColor = UiTheme.Surface,
            ForeColor = UiTheme.Text,
            BorderStyle = BorderStyle.None,
            CellBorderStyle = DataGridViewCellBorderStyle.SingleHorizontal,
            GridColor = UiTheme.Border,
            RowHeadersVisible = false,
            AllowUserToAddRows = false,
            AllowUserToDeleteRows = false,
            AllowUserToResizeRows = false,
            ReadOnly = true,
            SelectionMode = DataGridViewSelectionMode.FullRowSelect,
            MultiSelect = false,
            EnableHeadersVisualStyles = false,
            Font = UiTheme.FontSmall
        };

        grid.ColumnHeadersDefaultCellStyle.BackColor = UiTheme.SurfaceAlt;
        grid.ColumnHeadersDefaultCellStyle.ForeColor = UiTheme.Text;
        grid.ColumnHeadersDefaultCellStyle.Font = UiTheme.FontSection;
        grid.ColumnHeadersDefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleLeft;
        grid.ColumnHeadersHeight = 36;

        grid.DefaultCellStyle.BackColor = UiTheme.Surface;
        grid.DefaultCellStyle.ForeColor = UiTheme.Text;
        grid.DefaultCellStyle.SelectionBackColor = UiTheme.AccentDark;
        grid.DefaultCellStyle.SelectionForeColor = Color.White;

        grid.Columns.Add("Id", "Driver ID");
        grid.Columns.Add("Line", "Tuyến (Line)");
        grid.Columns.Add("Port", "Cổng COM");
        grid.Columns.Add("State", "Trạng thái");
        grid.Columns.Add("Pos", "Vị trí (vòng)");
        grid.Columns.Add("Vel", "Vận tốc (rpm)");
        grid.Columns.Add("Cmd", "Lệnh gần nhất");

        grid.Columns[0].Width = 100;
        grid.Columns[1].Width = 110;
        grid.Columns[2].Width = 110;
        grid.Columns[3].Width = 140;
        grid.Columns[4].Width = 130;
        grid.Columns[5].Width = 130;
        grid.Columns[6].AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill;

        return grid;
    }

    private void PopulateGridRows()
    {
        _grid.Rows.Clear();
        foreach (var axis in _state.Axes.OrderBy(a => a.Address.LinearIndex))
        {
            var line = _state.Lines[axis.Address.Line - 1];
            _grid.Rows.Add(
                axis.Address.DisplayId,
                $"Line {axis.Address.Line}",
                line.PortName,
                axis.State.ToString().ToUpperInvariant(),
                $"{axis.PositionRevolutions:0.000}",
                $"{axis.VelocityRpm} rpm",
                axis.LastCommand
            );
        }
    }

    private void UpdateDashboard()
    {
        if (!Visible) return;

        var allAxes = _state.Axes.OrderBy(a => a.Address.LinearIndex).ToArray();
        _onlineMetric.Value = $"{_state.OnlineCount} / 64";
        _homedMetric.Value = $"{_state.HomedCount} / 64";
        _movingMetric.Value = $"{allAxes.Count(a => a.State == AxisMotionState.Moving || a.State == AxisMotionState.JoggingForward || a.State == AxisMotionState.JoggingReverse)} / 64";
        _alarmMetric.Value = _state.AlarmCount.ToString();
        _alarmMetric.ValueColor = _state.AlarmCount > 0 ? UiTheme.Error : UiTheme.Text;

        if (_grid.Rows.Count != allAxes.Length)
        {
            PopulateGridRows();
            return;
        }

        for (var i = 0; i < allAxes.Length; i++)
        {
            var axis = allAxes[i];
            var row = _grid.Rows[i];
            var line = _state.Lines[axis.Address.Line - 1];

            row.Cells[2].Value = line.PortName;
            row.Cells[3].Value = axis.State.ToString().ToUpperInvariant();
            row.Cells[4].Value = $"{axis.PositionRevolutions:0.000}";
            row.Cells[5].Value = $"{axis.VelocityRpm} rpm";
            row.Cells[6].Value = axis.LastCommand;

            row.Cells[3].Style.ForeColor = axis.State switch
            {
                AxisMotionState.Online => UiTheme.Online,
                AxisMotionState.Homed => UiTheme.Homed,
                AxisMotionState.Moving or AxisMotionState.Homing or AxisMotionState.JoggingForward or AxisMotionState.JoggingReverse => UiTheme.Warning,
                AxisMotionState.Alarm => UiTheme.Error,
                _ => UiTheme.Muted
            };
        }
    }
}
