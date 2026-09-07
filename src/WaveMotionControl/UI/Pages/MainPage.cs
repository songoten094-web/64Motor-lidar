using System.ComponentModel;
using WaveMotionControl.Models;
using WaveMotionControl.Services;
using WaveMotionControl.State;
using WaveMotionControl.UI.Controls;

namespace WaveMotionControl.UI.Pages;

[DesignerCategory("UserControl")]
public partial class MainPage : UserControl
{
    private readonly ApplicationState _state;
    private readonly IRs485Service _service;
    private readonly Dictionary<int, ComboBox> _portBoxes = new();
    private readonly Dictionary<int, ComboBox> _baudBoxes = new();
    private readonly Dictionary<int, Button> _connectButtons = new();
    private readonly MetricCard _lineMetric = new("Line kết nối");
    private readonly MetricCard _onlineMetric = new("Driver online");
    private readonly MetricCard _homedMetric = new("Đã lấy gốc");
    private readonly MetricCard _alarmMetric = new("Alarm");
    private readonly AxisGridControl _axisGrid;
    private readonly ComboBox _homeTarget;
    private readonly LogView _logView;

    public MainPage() : this(new ApplicationState(), new DemoRs485Service(new ApplicationState()))
    {
    }

    public MainPage(ApplicationState state, IRs485Service service)
    {
        _state = state;
        _service = service;
        _axisGrid = new AxisGridControl(state);
        _logView = new LogView(state);
        _homeTarget = UiTheme.ComboBox();

        InitializeComponent();

        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 3,
            RowCount = 1,
            BackColor = UiTheme.Background,
            Padding = new Padding(0)
        };
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 370));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 410));

        root.Controls.Add(BuildConnectionPanel(), 0, 0);
        root.Controls.Add(BuildCenterPanel(), 1, 0);
        root.Controls.Add(BuildLogPanel(), 2, 0);
        Controls.Add(root);

        PopulateHomeTargets();
        RefreshSystemComPorts();
        _axisGrid.AxisSelected += axis =>
        {
            _state.SelectedAxis = axis;
            _homeTarget.SelectedItem = axis.DisplayId;
        };

        _state.StateChanged += OnStateChanged;
        Disposed += (_, _) => _state.StateChanged -= OnStateChanged;
        UpdateView();
    }

    private Control BuildConnectionPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0, 0, 10, 0);

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 7,
            ColumnCount = 1,
            BackColor = Color.Transparent
        };
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 58));
        for (var i = 0; i < 4; i++) layout.RowStyles.Add(new RowStyle(SizeType.Percent, 25));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 54));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 4));

        var headerBar = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1 };
        headerBar.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70));
        headerBar.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
        var title = BuildSectionHeader("KẾT NỐI RS485", "4 cổng COM phần cứng");
        var refreshBtn = UiTheme.Button("REFRESH");
        refreshBtn.Height = 32;
        refreshBtn.Font = UiTheme.FontSmall;
        refreshBtn.Click += (_, _) => RefreshSystemComPorts();
        headerBar.Controls.Add(title, 0, 0);
        headerBar.Controls.Add(refreshBtn, 1, 0);
        layout.Controls.Add(headerBar, 0, 0);

        for (var line = 1; line <= 4; line++)
        {
            layout.Controls.Add(BuildLineCard(line), 0, line);
        }

        var buttons = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1, Padding = new Padding(0, 8, 0, 0) };
        buttons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        buttons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        var connectAll = UiTheme.Button("CONNECT ALL", primary: true);
        var disconnectAll = UiTheme.Button("DISCONNECT");
        connectAll.Dock = DockStyle.Fill;
        disconnectAll.Dock = DockStyle.Fill;
        connectAll.Click += async (_, _) => await ConnectAllAsync();
        disconnectAll.Click += async (_, _) => await DisconnectAllAsync();
        buttons.Controls.Add(connectAll, 0, 0);
        buttons.Controls.Add(disconnectAll, 1, 0);
        layout.Controls.Add(buttons, 0, 5);

        card.Controls.Add(layout);
        return card;
    }

    private Control BuildLineCard(int line)
    {
        var panel = new Panel
        {
            Dock = DockStyle.Fill,
            BackColor = UiTheme.SurfaceAlt,
            Margin = new Padding(0, 5, 0, 5),
            Padding = new Padding(10),
            BorderStyle = BorderStyle.FixedSingle
        };

        var grid = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 2 };
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 54));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 120));
        grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
        grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50));

        var lineLabel = new Label
        {
            Text = $"L{line}",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleCenter,
            Font = new Font("Segoe UI Semibold", 14F, FontStyle.Bold),
            BackColor = UiTheme.Background,
            ForeColor = UiTheme.Text,
            Margin = new Padding(0, 0, 8, 0)
        };
        grid.Controls.Add(lineLabel, 0, 0);
        grid.SetRowSpan(lineLabel, 2);

        var port = UiTheme.ComboBox();
        port.Dock = DockStyle.Fill;
        var baud = UiTheme.ComboBox();
        baud.Dock = DockStyle.Fill;
        baud.Items.AddRange(new object[] { "9600", "19200", "38400", "115200" });
        baud.SelectedItem = "115200";

        var status = UiTheme.Label("0/16 online", UiTheme.FontSmall, UiTheme.Muted);
        status.Name = $"LineStatus{line}";
        status.Dock = DockStyle.Fill;
        status.TextAlign = ContentAlignment.MiddleLeft;
        status.AutoSize = false;

        var connect = UiTheme.Button("CONNECT");
        connect.Dock = DockStyle.Fill;
        connect.Font = new Font("Segoe UI Semibold", 9.5F, FontStyle.Bold);
        connect.Click += async (_, _) => await ToggleLineAsync(line);

        grid.Controls.Add(port, 1, 0);
        grid.Controls.Add(baud, 1, 1);
        grid.Controls.Add(connect, 2, 0);
        grid.Controls.Add(status, 2, 1);

        _portBoxes[line] = port;
        _baudBoxes[line] = baud;
        _connectButtons[line] = connect;

        panel.Controls.Add(grid);
        return panel;
    }

    private void RefreshSystemComPorts()
    {
        var realPorts = System.IO.Ports.SerialPort.GetPortNames().Distinct().OrderBy(p => p).ToArray();
        var allPorts = realPorts.Length > 0
            ? realPorts.Concat(Enumerable.Range(1, 16).Select(i => $"COM{i}")).Distinct().ToArray()
            : Enumerable.Range(1, 32).Select(i => $"COM{i}").ToArray();

        for (var line = 1; line <= 4; line++)
        {
            if (_portBoxes.TryGetValue(line, out var box))
            {
                var selected = box.SelectedItem?.ToString();
                box.Items.Clear();
                box.Items.AddRange(allPorts.Cast<object>().ToArray());
                if (selected != null && box.Items.Contains(selected))
                {
                    box.SelectedItem = selected;
                }
                else if (realPorts.Length >= line)
                {
                    box.SelectedItem = realPorts[line - 1];
                }
                else if (box.Items.Count > 0)
                {
                    box.SelectedIndex = Math.Min(line - 1, box.Items.Count - 1);
                }
            }
        }
        _state.WriteLog(LogLevel.Info, realPorts.Length > 0
            ? $"[Cổng COM Thật] Đã phát hiện {realPorts.Length} cổng COM phần cứng: {string.Join(", ", realPorts)}"
            : "[Cổng COM] Không tìm thấy thiết bị phần cứng cắm cổng COM. Đã nạp danh sách COM1..COM32 mặc định.");
    }

    private Control BuildCenterPanel()
    {
        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 3,
            ColumnCount = 1,
            BackColor = UiTheme.Background,
            Margin = new Padding(0, 0, 10, 0)
        };
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 94));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 134));

        var metrics = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 4, RowCount = 1 };
        for (var i = 0; i < 4; i++) metrics.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25));
        metrics.Controls.Add(_lineMetric, 0, 0);
        metrics.Controls.Add(_onlineMetric, 1, 0);
        metrics.Controls.Add(_homedMetric, 2, 0);
        metrics.Controls.Add(_alarmMetric, 3, 0);

        var gridCard = UiTheme.Card(8);
        gridCard.Dock = DockStyle.Fill;
        gridCard.Margin = new Padding(0, 8, 0, 8);
        var gridLayout = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 2, ColumnCount = 1 };
        gridLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 54));
        gridLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        gridLayout.Controls.Add(BuildSectionHeader("TRẠNG THÁI 64 DRIVER", "ID: 1.1–1.16, 2.1–2.16, 3.1–3.16, 4.1–4.16"), 0, 0);
        gridLayout.Controls.Add(_axisGrid, 0, 1);
        gridCard.Controls.Add(gridLayout);

        var homeCard = UiTheme.Card();
        homeCard.Dock = DockStyle.Fill;
        var homeGrid = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 4, RowCount = 2 };
        homeGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 42));
        homeGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 24));
        homeGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 20));
        homeGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 14));
        homeGrid.RowStyles.Add(new RowStyle(SizeType.Absolute, 50));
        homeGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var header = BuildSectionHeader("GO HOME", "Chọn một hoặc toàn bộ driver");
        homeGrid.Controls.Add(header, 0, 0);
        homeGrid.SetColumnSpan(header, 4);

        _homeTarget.Dock = DockStyle.Fill;
        var scope = UiTheme.ComboBox();
        scope.Items.AddRange(new object[] { "Driver được chọn", "Toàn bộ line", "Tất cả 64 driver" });
        scope.SelectedIndex = 0;
        scope.Dock = DockStyle.Fill;
        var start = UiTheme.Button("HOME START", primary: true);
        start.Dock = DockStyle.Fill;
        var stop = UiTheme.Button("STOP", danger: true);
        stop.Dock = DockStyle.Fill;

        start.Click += async (_, _) => await HomeAsync(scope.SelectedIndex);
        stop.Click += async (_, _) => await _service.StopAllAsync(true);

        homeGrid.Controls.Add(_homeTarget, 0, 1);
        homeGrid.Controls.Add(scope, 1, 1);
        homeGrid.Controls.Add(start, 2, 1);
        homeGrid.Controls.Add(stop, 3, 1);
        homeCard.Controls.Add(homeGrid);

        root.Controls.Add(metrics, 0, 0);
        root.Controls.Add(gridCard, 0, 1);
        root.Controls.Add(homeCard, 0, 2);
        return root;
    }

    private Control BuildLogPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        var layout = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 3, ColumnCount = 1 };
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 58));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 42));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        layout.Controls.Add(BuildSectionHeader("SYSTEM LOG", "Kết nối, Homing và trạng thái driver"), 0, 0);

        var bar = new FlowLayoutPanel { Dock = DockStyle.Fill, FlowDirection = FlowDirection.RightToLeft, BackColor = Color.Transparent };
        var clear = UiTheme.Button("CLEAR");
        clear.Width = 90;
        clear.Click += (_, _) => _logView.ClearLog();
        bar.Controls.Add(clear);
        layout.Controls.Add(bar, 0, 1);
        layout.Controls.Add(_logView, 0, 2);
        card.Controls.Add(layout);
        return card;
    }

    private static Control BuildSectionHeader(string title, string subtitle)
    {
        var panel = new Panel { Dock = DockStyle.Fill, BackColor = Color.Transparent };
        var titleLabel = UiTheme.Label(title, UiTheme.FontSection, UiTheme.Text);
        titleLabel.Location = new Point(0, 3);
        var subtitleLabel = UiTheme.Label(subtitle, UiTheme.FontSmall, UiTheme.Muted);
        subtitleLabel.Location = new Point(0, 27);
        panel.Controls.Add(titleLabel);
        panel.Controls.Add(subtitleLabel);
        return panel;
    }

    private void PopulateHomeTargets()
    {
        _homeTarget.Items.Clear();
        _homeTarget.Items.Add("TẤT CẢ 64 DRIVER");
        foreach (var axis in AxisAddress.All()) _homeTarget.Items.Add(axis.DisplayId);
        _homeTarget.SelectedItem = _state.SelectedAxis.DisplayId;
    }

    private async Task ToggleLineAsync(int line)
    {
        try
        {
            var connection = _state.Lines[line - 1];
            if (connection.IsConnected)
            {
                await _service.DisconnectLineAsync(line);
            }
            else
            {
                var port = _portBoxes[line].SelectedItem?.ToString() ?? $"COM{line}";
                var baud = int.Parse(_baudBoxes[line].SelectedItem?.ToString() ?? "115200");
                await _service.ConnectLineAsync(line, port, baud);
            }
        }
        catch (Exception ex)
        {
            _state.WriteLog(LogLevel.Error, ex.Message);
        }
    }

    private async Task ConnectAllAsync()
    {
        for (var line = 1; line <= 4; line++)
        {
            if (_state.Lines[line - 1].IsConnected) continue;
            await ToggleLineAsync(line);
        }
    }

    private async Task DisconnectAllAsync()
    {
        for (var line = 1; line <= 4; line++)
        {
            if (!_state.Lines[line - 1].IsConnected) continue;
            await ToggleLineAsync(line);
        }
    }

    private async Task HomeAsync(int scopeIndex)
    {
        try
        {
            IEnumerable<AxisAddress> targets;
            var selectedText = _homeTarget.SelectedItem?.ToString();
            if (selectedText == "TẤT CẢ 64 DRIVER" || scopeIndex == 2)
            {
                targets = AxisAddress.All();
            }
            else if (AxisAddress.TryParse(selectedText, out var selected))
            {
                targets = scopeIndex == 1
                    ? AxisAddress.All().Where(a => a.Line == selected.Line)
                    : new[] { selected };
            }
            else
            {
                targets = new[] { _state.SelectedAxis };
            }

            await _service.HomeAsync(targets);
        }
        catch (Exception ex)
        {
            _state.WriteLog(LogLevel.Error, ex.Message);
        }
    }

    private void OnStateChanged(object? sender, EventArgs e)
    {
        if (InvokeRequired)
        {
            BeginInvoke(new Action(UpdateView));
            return;
        }
        UpdateView();
    }

    private void UpdateView()
    {
        _lineMetric.Value = $"{_state.Lines.Count(l => l.IsConnected)} / 4";
        _onlineMetric.Value = $"{_state.OnlineCount} / 64";
        _homedMetric.Value = $"{_state.HomedCount} / 64";
        _alarmMetric.Value = _state.AlarmCount.ToString();
        _alarmMetric.ValueColor = _state.AlarmCount > 0 ? UiTheme.Error : UiTheme.Text;

        for (var line = 1; line <= 4; line++)
        {
            var connection = _state.Lines[line - 1];
            _connectButtons[line].Text = connection.IsConnected ? "DISCONNECT" : "CONNECT";
            _connectButtons[line].FlatAppearance.BorderColor = connection.IsConnected ? UiTheme.Online : UiTheme.Border;
            _portBoxes[line].Enabled = !connection.IsConnected;
            _baudBoxes[line].Enabled = !connection.IsConnected;
            var status = FindControlRecursive(this, $"LineStatus{line}") as Label;
            if (status is not null)
            {
                var online = _state.GetAxesForLine(line).Count(a => a.IsOnline);
                status.Text = $"{online}/16 online";
                status.ForeColor = online == 16 ? UiTheme.Online : UiTheme.Muted;
            }
        }
    }

    private static Control? FindControlRecursive(Control parent, string name)
    {
        foreach (Control child in parent.Controls)
        {
            if (child.Name == name) return child;
            var found = FindControlRecursive(child, name);
            if (found is not null) return found;
        }
        return null;
    }

    private void MainPage_Load(object sender, EventArgs e)
    {

    }
}
