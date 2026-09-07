using System.ComponentModel;
using WaveMotionControl.Models;
using WaveMotionControl.Services;
using WaveMotionControl.State;
using WaveMotionControl.UI.Controls;

namespace WaveMotionControl.UI.Pages;

[DesignerCategory("UserControl")]
public partial class ManualPage : UserControl
{
    // Kích thước thiết kế tối thiểu. Khi cửa sổ nhỏ hơn, trang sẽ xuất hiện
    // thanh cuộn thay vì ép các control đè lên nhau.
    private const int MinimumCanvasWidth = 1540;
    private const int MinimumCanvasHeight = 1180;

    private readonly ApplicationState _state;
    private readonly IRs485Service _service;
    private readonly AxisGridControl _axisGrid;
    private readonly LogView _logView;
    private readonly TextBox _axisInput;
    private readonly Label _selectedId;
    private readonly Label _selectedInfo;
    private readonly Label _modeValue;
    private readonly Label _speedValue;
    private readonly Label _commandValue;
    private readonly NumericUpDown _jogSpeed;
    private readonly NumericUpDown _jogAcc;
    private readonly NumericUpDown _jogDec;
    private readonly NumericUpDown _revolutions;
    private readonly NumericUpDown _moveSpeed;
    private readonly NumericUpDown _pulsePerRev;
    private readonly ComboBox _direction;
    private readonly Label _pulseValue;
    private readonly Label _highValue;
    private readonly Label _lowValue;
    private readonly Label _timeValue;
    private readonly ProgressBar _moveProgress;
    private readonly System.Windows.Forms.Timer _progressTimer;

    private readonly NumericUpDown _uniformCrankRadius;
    private readonly NumericUpDown _uniformRodLength;
    private readonly NumericUpDown _uniformOffset;
    private readonly NumericUpDown _uniformSliderSpeed;
    private readonly NumericUpDown _uniformCurrent;
    private readonly NumericUpDown _uniformAcceleration;
    private readonly NumericUpDown _uniformDeceleration;
    private readonly ComboBox _uniformHomePoint;
    private readonly ComboBox _uniformMotorDirection;
    private readonly Label _uniformStrokeValue;
    private readonly Label _uniformCycleValue;
    private readonly Label _uniformRpmValue;
    private readonly Label _uniformHomeReferenceValue;
    private AxisAddress? _uniformRunningAxis;

    private CancellationTokenSource? _moveCts;
    private DateTime _moveStarted;
    private double _estimatedSeconds;

    // Constructor dành cho WinForms Designer. ApplicationState và service phải
    // dùng chung một state, tránh trạng thái giao diện và mô phỏng bị lệch nhau.
    public ManualPage() : this(new ApplicationState())
    {
    }

    private ManualPage(ApplicationState state)
        : this(state, new DemoRs485Service(state))
    {
    }

    public ManualPage(ApplicationState state, IRs485Service service)
    {
        ArgumentNullException.ThrowIfNull(state);
        ArgumentNullException.ThrowIfNull(service);

        _state = state;
        _service = service;

        SuspendLayout();

        AutoScaleMode = AutoScaleMode.Dpi;
        AutoScroll = true;
        AutoScrollMinSize = new Size(MinimumCanvasWidth + 24, MinimumCanvasHeight + 24);
        BackColor = UiTheme.Background;
        Padding = new Padding(12);

        _axisGrid = new AxisGridControl(state)
        {
            Dock = DockStyle.Fill,
            Margin = new Padding(0)
        };

        _logView = new LogView(state)
        {
            Dock = DockStyle.Fill,
            Margin = new Padding(0)
        };

        _axisInput = UiTheme.TextBox("1.1");
        _selectedId = UiTheme.Label(
            "1.1",
            new Font("Segoe UI Semibold", 25F, FontStyle.Bold));
        _selectedInfo = UiTheme.Label(
            "Line 1 · Slave 1 · COM1",
            UiTheme.FontSmall,
            UiTheme.Muted);
        _modeValue = UiTheme.Label("IDLE", UiTheme.FontSection);
        _speedValue = UiTheme.Label("0 rpm", UiTheme.FontSection);
        _commandValue = UiTheme.Label("—", UiTheme.FontSection);

        _jogSpeed = UiTheme.Numeric(60, 1, 5000, 1);
        _jogAcc = UiTheme.Numeric(200, 1, 5000, 1);
        _jogDec = UiTheme.Numeric(200, 1, 5000, 1);
        _revolutions = UiTheme.Numeric(1, 0.001M, 100000, 0.001M, 3);
        _moveSpeed = UiTheme.Numeric(200, 1, 5000, 1);
        _pulsePerRev = UiTheme.Numeric(10000, 200, 51200, 1);

        _uniformCrankRadius = UiTheme.Numeric(50, 1, 1000, 0.1M, 1);
        _uniformRodLength = UiTheme.Numeric(100, 1, 2000, 0.1M, 1);
        _uniformOffset = UiTheme.Numeric(15, 0, 1000, 0.1M, 1);
        _uniformSliderSpeed = UiTheme.Numeric(20, 5, 1000, 1);
        _uniformCurrent = UiTheme.Numeric(3.0M, 0.5M, 6.0M, 0.1M, 1);
        _uniformAcceleration = UiTheme.Numeric(1000, 1, 10000, 10);
        _uniformDeceleration = UiTheme.Numeric(1000, 1, 10000, 10);

        _uniformHomePoint = UiTheme.ComboBox();
        _uniformHomePoint.Items.AddRange(new object[]
        {
            "Đầu ngoài / x lớn nhất",
            "Đầu trong / x nhỏ nhất"
        });
        _uniformHomePoint.SelectedIndex = 0;

        _uniformMotorDirection = UiTheme.ComboBox();
        _uniformMotorDirection.Items.AddRange(new object[]
        {
            "Forward / chiều dương",
            "Reverse / chiều âm"
        });
        _uniformMotorDirection.SelectedIndex = 0;

        _uniformStrokeValue = CreateCalculationLabel("101.551 mm");
        _uniformCycleValue = CreateCalculationLabel("10.16 s/vòng");
        _uniformRpmValue = CreateCalculationLabel("— rpm");
        _uniformHomeReferenceValue = CreateCalculationLabel("5.739° · đặt vị trí 0");

        _direction = UiTheme.ComboBox();
        _direction.Items.AddRange(new object[] { "Tiến / CW", "Lùi / CCW" });
        _direction.SelectedIndex = 0;

        _pulseValue = CreateCalculationLabel("10,000 p");
        _highValue = CreateCalculationLabel("0x0000");
        _lowValue = CreateCalculationLabel("0x2710");
        _timeValue = CreateCalculationLabel("0.30 s");

        _moveProgress = new ProgressBar
        {
            Dock = DockStyle.Fill,
            Minimum = 0,
            Maximum = 100,
            Value = 0,
            Style = ProgressBarStyle.Continuous,
            Margin = new Padding(0, 4, 0, 0)
        };

        var canvas = new Panel
        {
            Name = "ManualDesignCanvas",
            Location = new Point(Padding.Left, Padding.Top),
            Size = new Size(MinimumCanvasWidth, MinimumCanvasHeight),
            BackColor = UiTheme.Background,
            Margin = new Padding(0),
            Padding = new Padding(0),
            Anchor = AnchorStyles.Top | AnchorStyles.Left
        };

        var root = new TableLayoutPanel
        {
            Name = "ManualRootLayout",
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = 1,
            BackColor = UiTheme.Background,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        // Cột trái đủ rộng cho bảng 64 driver; cột phải đủ rộng cho status/log.
        // Cột giữa tự nhận toàn bộ phần không gian còn lại.
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 440F));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 340F));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        root.Controls.Add(BuildAxisPanel(), 0, 0);
        root.Controls.Add(BuildControlPanel(), 1, 0);
        root.Controls.Add(BuildStatusPanel(), 2, 0);

        canvas.Controls.Add(root);
        Controls.Add(canvas);

        void ResizeCanvas()
        {
            var availableWidth = Math.Max(0, ClientSize.Width - Padding.Horizontal);
            var availableHeight = Math.Max(0, ClientSize.Height - Padding.Vertical);

            canvas.Size = new Size(
                Math.Max(MinimumCanvasWidth, availableWidth),
                Math.Max(MinimumCanvasHeight, availableHeight));

            AutoScrollMinSize = new Size(
                canvas.Width + Padding.Horizontal,
                canvas.Height + Padding.Vertical);
        }

        Resize += (_, _) => ResizeCanvas();
        ResizeCanvas();

        _axisGrid.AxisSelected += axis => SelectAxis(axis);
        _state.StateChanged += OnStateChanged;
        Disposed += (_, _) => _state.StateChanged -= OnStateChanged;

        foreach (var numeric in new[] { _revolutions, _moveSpeed, _pulsePerRev })
        {
            numeric.ValueChanged += (_, _) => UpdateMoveCalculation();
        }

        _direction.SelectedIndexChanged += (_, _) => UpdateMoveCalculation();

        foreach (var numeric in new[]
                 {
                     _uniformCrankRadius,
                     _uniformRodLength,
                     _uniformOffset,
                     _uniformSliderSpeed,
                     _uniformCurrent,
                     _uniformAcceleration,
                     _uniformDeceleration
                 })
        {
            numeric.ValueChanged += (_, _) => UpdateUniformCalculation();
        }

        _uniformHomePoint.SelectedIndexChanged +=
            (_, _) => UpdateUniformCalculation();
        _uniformMotorDirection.SelectedIndexChanged +=
            (_, _) => UpdateUniformCalculation();

        _progressTimer = new System.Windows.Forms.Timer { Interval = 80 };
        _progressTimer.Tick += (_, _) => UpdateProgress();

        SelectAxis(new AxisAddress(1, 1));
        UpdateMoveCalculation();
        UpdateUniformCalculation();

        ResumeLayout(performLayout: true);
    }

    private Control BuildAxisPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0, 0, 10, 0);
        card.Padding = new Padding(12);
        card.MinimumSize = new Size(420, 0);

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 4,
            ColumnCount = 1,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 58F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 150F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 38F));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        layout.Controls.Add(
            BuildHeader("CHỌN DRIVER", "Nhập ID dạng 1.1 đến 4.16"),
            0,
            0);

        var selector = new Panel
        {
            Dock = DockStyle.Fill,
            BackColor = Color.Transparent,
            Padding = new Padding(0, 8, 0, 8),
            Margin = new Padding(0)
        };

        var selectorGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 2,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        selectorGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 72F));
        selectorGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 28F));
        selectorGrid.RowStyles.Add(new RowStyle(SizeType.Absolute, 42F));
        selectorGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        _axisInput.Dock = DockStyle.Fill;
        _axisInput.Font = new Font("Segoe UI Semibold", 13F, FontStyle.Bold);
        _axisInput.Margin = new Padding(0, 0, 6, 0);

        var selectButton = UiTheme.Button("CHỌN", primary: true);
        selectButton.Dock = DockStyle.Fill;
        selectButton.Margin = new Padding(0);
        selectButton.Click += (_, _) => ParseAndSelectAxis();

        _axisInput.KeyDown += (_, e) =>
        {
            if (e.KeyCode != Keys.Enter)
            {
                return;
            }

            ParseAndSelectAxis();
            e.SuppressKeyPress = true;
        };

        var selectedPanel = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            BackColor = UiTheme.SurfaceAlt,
            Padding = new Padding(8),
            Margin = new Padding(0, 8, 0, 0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        selectedPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 88F));
        selectedPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        selectedPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        _selectedId.Dock = DockStyle.Fill;
        _selectedId.TextAlign = ContentAlignment.MiddleCenter;
        _selectedId.AutoSize = false;
        _selectedId.BackColor = UiTheme.Background;
        _selectedId.Margin = new Padding(0, 0, 8, 0);

        _selectedInfo.Dock = DockStyle.Fill;
        _selectedInfo.TextAlign = ContentAlignment.MiddleLeft;
        _selectedInfo.AutoSize = false;
        _selectedInfo.AutoEllipsis = true;
        _selectedInfo.Margin = new Padding(0);

        selectedPanel.Controls.Add(_selectedId, 0, 0);
        selectedPanel.Controls.Add(_selectedInfo, 1, 0);

        selectorGrid.Controls.Add(_axisInput, 0, 0);
        selectorGrid.Controls.Add(selectButton, 1, 0);
        selectorGrid.Controls.Add(selectedPanel, 0, 1);
        selectorGrid.SetColumnSpan(selectedPanel, 2);

        selector.Controls.Add(selectorGrid);
        layout.Controls.Add(selector, 0, 1);

        var listTitle = UiTheme.Label(
            "DANH SÁCH 64 DRIVER",
            UiTheme.FontSection,
            UiTheme.Muted);
        listTitle.Dock = DockStyle.Fill;
        listTitle.TextAlign = ContentAlignment.MiddleLeft;
        listTitle.AutoSize = false;
        listTitle.Margin = new Padding(0);

        layout.Controls.Add(listTitle, 0, 2);
        layout.Controls.Add(_axisGrid, 0, 3);
        card.Controls.Add(layout);

        return card;
    }

    private Control BuildControlPanel()
    {
        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 3,
            ColumnCount = 1,
            BackColor = UiTheme.Background,
            Margin = new Padding(0, 0, 10, 0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            MinimumSize = new Size(700, 0)
        };

        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 300F));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 430F));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        root.Controls.Add(BuildJogPanel(), 0, 0);
        root.Controls.Add(BuildMovePanel(), 0, 1);
        root.Controls.Add(BuildUniformSliderPanel(), 0, 2);

        return root;
    }

    private Control BuildJogPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0, 0, 0, 7);
        card.Padding = new Padding(12);

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 3,
            ColumnCount = 1,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 54F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 72F));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        layout.Controls.Add(
            BuildHeader("JOG THỦ CÔNG", "Giữ nút để chạy, thả nút để dừng"),
            0,
            0);

        var paramsGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 3,
            RowCount = 1,
            Padding = new Padding(0, 4, 0, 4),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        for (var i = 0; i < 3; i++)
        {
            paramsGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.333F));
        }

        paramsGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        paramsGrid.Controls.Add(LabeledControl("Tốc độ JOG (rpm)", _jogSpeed), 0, 0);
        paramsGrid.Controls.Add(LabeledControl("Gia tốc", _jogAcc), 1, 0);
        paramsGrid.Controls.Add(LabeledControl("Giảm tốc", _jogDec), 2, 0);
        layout.Controls.Add(paramsGrid, 0, 1);

        var buttonGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 3,
            RowCount = 1,
            Padding = new Padding(0, 5, 0, 0),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        buttonGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 38F));
        buttonGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 24F));
        buttonGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 38F));
        buttonGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var reverse = UiTheme.Button("◀  JOG LÙI\r\nCCW / JOG−");
        var forward = UiTheme.Button("JOG TIẾN  ▶\r\nCW / JOG+");

        reverse.Dock = DockStyle.Fill;
        forward.Dock = DockStyle.Fill;
        reverse.Margin = new Padding(0, 0, 6, 0);
        forward.Margin = new Padding(6, 0, 0, 0);
        reverse.Font = new Font("Segoe UI Semibold", 11F, FontStyle.Bold);
        forward.Font = new Font("Segoe UI Semibold", 11F, FontStyle.Bold);

        BindJogButton(reverse, JogDirection.Reverse);
        BindJogButton(forward, JogDirection.Forward);

        var center = new Label
        {
            Name = "JogCenterLabel",
            Text = "MOTOR\r\nSTOPPED",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleCenter,
            BackColor = UiTheme.Background,
            ForeColor = UiTheme.Muted,
            Font = UiTheme.FontSection,
            Margin = new Padding(0),
            AutoSize = false
        };

        buttonGrid.Controls.Add(reverse, 0, 0);
        buttonGrid.Controls.Add(center, 1, 0);
        buttonGrid.Controls.Add(forward, 2, 0);
        layout.Controls.Add(buttonGrid, 0, 2);
        card.Controls.Add(layout);

        return card;
    }

    private Control BuildMovePanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0, 7, 0, 0);
        card.Padding = new Padding(12);

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 5,
            ColumnCount = 1,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 52F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 74F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 74F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 52F));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        layout.Controls.Add(
            BuildHeader("CHẠY THEO SỐ VÒNG", "Chạy vị trí tương đối theo số vòng quay"),
            0,
            0);

        var paramsGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 5,
            RowCount = 1,
            Padding = new Padding(0, 4, 0, 4),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        for (var i = 0; i < 4; i++)
        {
            paramsGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25F));
        }

        paramsGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        paramsGrid.Controls.Add(LabeledControl("Số vòng", _revolutions), 0, 0);
        paramsGrid.Controls.Add(LabeledControl("Chiều quay", _direction), 1, 0);
        paramsGrid.Controls.Add(LabeledControl("Tốc độ (rpm)", _moveSpeed), 2, 0);
        paramsGrid.Controls.Add(LabeledControl("Pulse / vòng", _pulsePerRev), 3, 0);
        layout.Controls.Add(paramsGrid, 0, 1);

        var calcGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = 1,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        for (var i = 0; i < 4; i++)
        {
            calcGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25F));
        }

        calcGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        calcGrid.Controls.Add(ValueCard("Vị trí tương đối", _pulseValue), 0, 0);
        calcGrid.Controls.Add(ValueCard("Position High", _highValue), 1, 0);
        calcGrid.Controls.Add(ValueCard("Position Low", _lowValue), 2, 0);
        calcGrid.Controls.Add(ValueCard("Thời gian dự kiến", _timeValue), 3, 0);
        layout.Controls.Add(calcGrid, 0, 2);

        var actions = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            Padding = new Padding(0, 5, 0, 5),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        actions.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        actions.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        actions.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var start = UiTheme.Button("START MOVE", primary: true);
        var stop = UiTheme.Button("QUICK STOP", danger: true);
        start.Dock = DockStyle.Fill;
        stop.Dock = DockStyle.Fill;
        start.Margin = new Padding(0, 0, 5, 0);
        stop.Margin = new Padding(5, 0, 0, 0);
        start.Click += async (_, _) => await StartMoveAsync();
        stop.Click += async (_, _) => await StopMoveAsync();

        actions.Controls.Add(start, 0, 0);
        actions.Controls.Add(stop, 1, 0);
        layout.Controls.Add(actions, 0, 3);

        var progressWrap = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 2,
            ColumnCount = 1,
            BackColor = UiTheme.SurfaceAlt,
            Padding = new Padding(8, 4, 8, 6),
            Margin = new Padding(0, 4, 0, 0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        progressWrap.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        progressWrap.RowStyles.Add(new RowStyle(SizeType.Absolute, 22F));
        progressWrap.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var progressLabel = UiTheme.Label(
            "Tiến độ lệnh MOVE",
            UiTheme.FontSmall,
            UiTheme.Muted);
        progressLabel.Dock = DockStyle.Fill;
        progressLabel.TextAlign = ContentAlignment.MiddleLeft;
        progressLabel.AutoSize = false;
        progressLabel.Margin = new Padding(0);

        progressWrap.Controls.Add(progressLabel, 0, 0);
        progressWrap.Controls.Add(_moveProgress, 0, 1);
        layout.Controls.Add(progressWrap, 0, 4);
        card.Controls.Add(layout);

        return card;
    }

    private Control BuildUniformSliderPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0, 7, 0, 0);
        card.Padding = new Padding(12);

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 6,
            ColumnCount = 1,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 54F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 76F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 54F));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        layout.Controls.Add(
            BuildHeader(
                "TEST CON TRƯỢT GẦN ĐỀU — 16 PR",
                "Ghi PR0..PR15 một lần; OVLP nối mượt tốc độ và driver tự Jump lặp nội bộ"),
            0,
            0);

        var geometryGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = 1,
            Padding = new Padding(0, 4, 0, 4),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        for (var index = 0; index < 4; index++)
        {
            geometryGrid.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 25F));
        }

        geometryGrid.RowStyles.Add(
            new RowStyle(SizeType.Percent, 100F));
        geometryGrid.Controls.Add(
            LabeledControl("Tay quay R (mm)", _uniformCrankRadius), 0, 0);
        geometryGrid.Controls.Add(
            LabeledControl("Thanh truyền L (mm)", _uniformRodLength), 1, 0);
        geometryGrid.Controls.Add(
            LabeledControl("Lệch tâm e (mm)", _uniformOffset), 2, 0);
        geometryGrid.Controls.Add(
            LabeledControl("Tốc độ con trượt (mm/s)", _uniformSliderSpeed), 3, 0);
        layout.Controls.Add(geometryGrid, 0, 1);

        var modeGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 5,
            RowCount = 1,
            Padding = new Padding(0, 4, 0, 4),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        for (var index = 0; index < 5; index++)
        {
            modeGrid.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 20F));
        }

        modeGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        modeGrid.Controls.Add(
            LabeledControl("Home đang ở", _uniformHomePoint), 0, 0);
        modeGrid.Controls.Add(
            LabeledControl("Chiều quay motor", _uniformMotorDirection), 1, 0);
        modeGrid.Controls.Add(
            LabeledControl("Dòng Peak test 16 PR (A)", _uniformCurrent), 2, 0);
        modeGrid.Controls.Add(
            LabeledControl("PR Acc (ms/1000rpm)", _uniformAcceleration), 3, 0);
        modeGrid.Controls.Add(
            LabeledControl("PR Dec (ms/1000rpm)", _uniformDeceleration), 4, 0);
        layout.Controls.Add(modeGrid, 0, 2);

        var summaryGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = 1,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        for (var index = 0; index < 4; index++)
        {
            summaryGrid.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 25F));
        }

        summaryGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        summaryGrid.Controls.Add(
            ValueCard("Hành trình hình học", _uniformStrokeValue), 0, 0);
        summaryGrid.Controls.Add(
            ValueCard("Chu kỳ dự kiến", _uniformCycleValue), 1, 0);
        summaryGrid.Controls.Add(
            ValueCard("Dải tốc độ motor", _uniformRpmValue), 2, 0);
        summaryGrid.Controls.Add(
            ValueCard("Gốc cơ khí chuẩn", _uniformHomeReferenceValue), 3, 0);
        layout.Controls.Add(summaryGrid, 0, 3);

        var actions = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 3,
            RowCount = 1,
            Padding = new Padding(0, 5, 0, 5),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        actions.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 28F));
        actions.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 46F));
        actions.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 26F));
        actions.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var setZero = UiTheme.Button("ĐẶT GỐC TẠI ĐÂY");
        var start = UiTheme.Button("GHI 16 PR + START LOOP", primary: true);
        var stop = UiTheme.Button("QUICK STOP TEST", danger: true);
        setZero.Dock = DockStyle.Fill;
        start.Dock = DockStyle.Fill;
        stop.Dock = DockStyle.Fill;
        setZero.Margin = new Padding(0, 0, 4, 0);
        start.Margin = new Padding(4, 0, 4, 0);
        stop.Margin = new Padding(4, 0, 0, 0);
        setZero.Click += async (_, _) =>
            await SetUniformMechanicalZeroAsync();
        start.Click += async (_, _) =>
            await StartUniformSliderTestAsync();
        stop.Click += async (_, _) =>
            await StopUniformSliderTestAsync();

        actions.Controls.Add(setZero, 0, 0);
        actions.Controls.Add(start, 1, 0);
        actions.Controls.Add(stop, 2, 0);
        layout.Controls.Add(actions, 0, 4);

        var note = UiTheme.Label(
            "Dòng test 16 PR được ghi vào 0x0191 khi START, giới hạn 0,5–6,0 A và không lưu EEPROM. " +
            "Gốc cơ khí phải đặt đúng điểm chết đã hiển thị; A-B-C thẳng hàng. " +
            "OVLP được bật, PR chỉ ghi RAM. Sau Quick Stop phải lấy gốc lại trước khi START.",
            UiTheme.FontSmall,
            UiTheme.Warning);
        note.Dock = DockStyle.Fill;
        note.AutoSize = false;
        note.TextAlign = ContentAlignment.MiddleLeft;
        note.Margin = new Padding(4, 4, 4, 0);
        layout.Controls.Add(note, 0, 5);

        card.Controls.Add(layout);
        return card;
    }

    private Control BuildStatusPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0);
        card.Padding = new Padding(12);
        card.MinimumSize = new Size(320, 0);

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 4,
            ColumnCount = 1,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 58F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 176F));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44F));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        layout.Controls.Add(
            BuildHeader("MANUAL STATUS", "Lệnh và phản hồi mô phỏng"),
            0,
            0);

        var statusGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 2,
            Padding = new Padding(0, 6, 0, 6),
            Margin = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        statusGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        statusGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        statusGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
        statusGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));

        var driver = UiTheme.Label("1.1", UiTheme.FontSection);
        driver.Name = "StatusDriver";

        statusGrid.Controls.Add(ValueCard("Driver", driver), 0, 0);
        statusGrid.Controls.Add(ValueCard("Mode", _modeValue), 1, 0);
        statusGrid.Controls.Add(ValueCard("Velocity", _speedValue), 0, 1);
        statusGrid.Controls.Add(ValueCard("Command", _commandValue), 1, 1);
        layout.Controls.Add(statusGrid, 0, 1);

        var clearBar = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            FlowDirection = FlowDirection.RightToLeft,
            WrapContents = false,
            BackColor = Color.Transparent,
            Margin = new Padding(0),
            Padding = new Padding(0, 4, 0, 4)
        };

        var clear = UiTheme.Button("CLEAR LOG");
        clear.Width = 110;
        clear.Height = 34;
        clear.Margin = new Padding(0);
        clear.Click += (_, _) => _logView.ClearLog();
        clearBar.Controls.Add(clear);

        layout.Controls.Add(clearBar, 0, 2);
        layout.Controls.Add(_logView, 0, 3);
        card.Controls.Add(layout);

        return card;
    }

    private void BindJogButton(Button button, JogDirection direction)
    {
        button.MouseDown += async (_, e) =>
        {
            if (e.Button != MouseButtons.Left)
            {
                return;
            }

            await StartJogAsync(direction);
        };

        button.MouseUp += async (_, _) => await StopJogAsync();
        button.MouseLeave += async (_, _) =>
        {
            if (Control.MouseButtons == MouseButtons.Left)
            {
                await StopJogAsync();
            }
        };
    }

    private async Task StartJogAsync(JogDirection direction)
    {
        try
        {
            if (_moveCts is not null)
            {
                _state.WriteLog(LogLevel.Warning, "Không thể JOG khi MOVE đang chạy.");
                return;
            }

            if (_uniformRunningAxis is not null)
            {
                _state.WriteLog(
                    LogLevel.Warning,
                    $"Test 16 PR đang chạy tại {_uniformRunningAxis.Value.DisplayId}. " +
                    "Hãy Quick Stop test trước khi JOG.");
                return;
            }

            await _service.StartJogAsync(
                _state.SelectedAxis,
                direction,
                (int)_jogSpeed.Value,
                (int)_jogAcc.Value,
                (int)_jogDec.Value);
        }
        catch (Exception ex)
        {
            _state.WriteLog(LogLevel.Error, ex.Message);
        }
    }

    private async Task StopJogAsync()
    {
        try
        {
            await _service.StopAxisAsync(_state.SelectedAxis);
        }
        catch (Exception ex)
        {
            _state.WriteLog(LogLevel.Error, ex.Message);
        }
    }

    private async Task StartMoveAsync()
    {
        if (_moveCts is not null)
        {
            _state.WriteLog(LogLevel.Warning, "Một lệnh MOVE đang chạy.");
            return;
        }

        if (_uniformRunningAxis is not null)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"Test 16 PR đang chạy tại {_uniformRunningAxis.Value.DisplayId}. " +
                "Hãy Quick Stop test trước khi MOVE.");
            return;
        }

        _moveCts = new CancellationTokenSource();
        var signedRevolutions =
            (double)_revolutions.Value *
            (_direction.SelectedIndex == 0 ? 1 : -1);

        _estimatedSeconds =
            Math.Abs(signedRevolutions) /
            Math.Max(1, (double)_moveSpeed.Value) *
            60;

        _moveStarted = DateTime.UtcNow;
        _moveProgress.Value = 0;
        _progressTimer.Start();

        try
        {
            await _service.MoveRelativeRevolutionsAsync(
                _state.SelectedAxis,
                signedRevolutions,
                (int)_moveSpeed.Value,
                (int)_pulsePerRev.Value,
                _moveCts.Token);

            _moveProgress.Value = 100;
        }
        catch (OperationCanceledException)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"Driver {_state.SelectedAxis}: MOVE đã bị hủy.");
        }
        catch (Exception ex)
        {
            _state.WriteLog(LogLevel.Error, ex.Message);
        }
        finally
        {
            _progressTimer.Stop();
            _moveCts.Dispose();
            _moveCts = null;
        }
    }

    private UniformSliderMotionSettings GetUniformSettings()
    {
        return new UniformSliderMotionSettings(
            CrankRadiusMm: (double)_uniformCrankRadius.Value,
            ConnectingRodLengthMm: (double)_uniformRodLength.Value,
            OffsetMm: (double)_uniformOffset.Value,
            SliderSpeedMmPerSecond: (double)_uniformSliderSpeed.Value,
            PeakCurrentAmps: (double)_uniformCurrent.Value,
            AccelerationMsPer1000Rpm: (int)_uniformAcceleration.Value,
            DecelerationMsPer1000Rpm: (int)_uniformDeceleration.Value,
            HomePoint: _uniformHomePoint.SelectedIndex == 0
                ? SliderHomePoint.OuterDeadCenter
                : SliderHomePoint.InnerDeadCenter,
            MotorDirection: _uniformMotorDirection.SelectedIndex == 0
                ? UniformSliderMotorDirection.Forward
                : UniformSliderMotorDirection.Reverse);
    }

    private void UpdateUniformCalculation()
    {
        try
        {
            var settings = GetUniformSettings();
            UniformSliderMotionPlan plan;

            if (_service is IUniformSliderMotionService uniformService)
            {
                plan = uniformService.PreviewUniformSliderMotion(
                    _state.SelectedAxis,
                    settings);
            }
            else
            {
                // Chỉ dùng khi chạy WinForms Designer/Demo service.
                plan = UniformSliderMotionPlanner.Build(
                    settings,
                    (int)_pulsePerRev.Value);
            }

            _uniformStrokeValue.Text = $"{plan.StrokeMm:0.###} mm";
            _uniformCycleValue.Text =
                $"{plan.DesiredCycleTimeSeconds:0.###} s/vòng";
            _uniformRpmValue.Text =
                $"{plan.MinimumSpeedRpm}–{plan.MaximumSpeedRpm} rpm";
            _uniformRpmValue.ForeColor = UiTheme.Text;

            var homeAngle = _uniformHomePoint.SelectedIndex == 0
                ? plan.OuterDeadCenterAngleDeg
                : plan.InnerDeadCenterAngleDeg;
            _uniformHomeReferenceValue.Text =
                $"{homeAngle:0.###}° · đặt vị trí 0";
            _uniformHomeReferenceValue.ForeColor = UiTheme.Online;
        }
        catch (Exception ex)
        {
            _uniformStrokeValue.Text = "Cấu hình lỗi";
            _uniformCycleValue.Text = "—";
            _uniformRpmValue.Text = ex.Message;
            _uniformRpmValue.ForeColor = UiTheme.Error;
            _uniformHomeReferenceValue.Text = "—";
            _uniformHomeReferenceValue.ForeColor = UiTheme.Error;
        }
    }

    private async Task SetUniformMechanicalZeroAsync()
    {
        if (_uniformRunningAxis is not null)
        {
            _state.WriteLog(
                LogLevel.Warning,
                "Hãy Quick Stop test 16 PR trước khi đặt lại gốc.");
            return;
        }

        if (_service is not IUniformSliderMotionService uniformService)
        {
            _state.WriteLog(
                LogLevel.Error,
                "Service hiện tại chưa hỗ trợ đặt gốc cơ khí cho 16 PR.");
            return;
        }

        var homeName = _uniformHomePoint.SelectedIndex == 0
            ? "đầu ngoài: A-B-C thẳng hàng, B nằm giữa A và C"
            : "đầu trong: B-A-C thẳng hàng, A nằm giữa B và C";

        var result = MessageBox.Show(
            $"Chỉ tiếp tục khi tay quay đã ở đúng {homeName}.\r\n" +
            $"Góc chuẩn đang hiển thị: {_uniformHomeReferenceValue.Text}.\r\n\r\n" +
            "Lệnh này sẽ đặt tọa độ hiện tại của driver thành 0 pulse.",
            "Xác nhận đặt gốc cơ khí",
            MessageBoxButtons.OKCancel,
            MessageBoxIcon.Warning);

        if (result != DialogResult.OK)
        {
            return;
        }

        try
        {
            await uniformService.SetUniformMechanicalZeroAsync(
                _state.SelectedAxis);
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Error,
                $"[UNIFORM ZERO] {_state.SelectedAxis.DisplayId}: {ex.Message}");
        }
    }

    private async Task StartUniformSliderTestAsync()
    {
        if (_moveCts is not null)
        {
            _state.WriteLog(
                LogLevel.Warning,
                "Không thể chạy test 16 PR khi MOVE đang chạy.");
            return;
        }

        if (_uniformRunningAxis is not null)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"Test 16 PR đã chạy tại {_uniformRunningAxis.Value.DisplayId}.");
            return;
        }

        if (_service is not IUniformSliderMotionService uniformService)
        {
            _state.WriteLog(
                LogLevel.Error,
                "Service hiện tại chưa hỗ trợ IUniformSliderMotionService.");
            return;
        }

        var address = _state.SelectedAxis;
        var axis = _state.GetAxis(address);

        if (axis.State != AxisMotionState.Homed)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"Driver {address.DisplayId} chưa HOME. " +
                "Hãy HOME thành công trước khi ghi và chạy 16 PR.");
            return;
        }

        try
        {
            await uniformService.StartUniformSliderMotionAsync(
                address,
                GetUniformSettings());

            _uniformRunningAxis = address;
            _state.WriteLog(
                LogLevel.Ok,
                $"Driver {address.DisplayId}: test con trượt gần đều đã chạy. " +
                "Driver tự Jump PR nội bộ; máy tính không truyền vị trí liên tục.");
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Error,
                $"[UNIFORM PR] {address.DisplayId}: {ex.Message}");
        }
    }

    private async Task StopUniformSliderTestAsync()
    {
        var address = _uniformRunningAxis ?? _state.SelectedAxis;

        try
        {
            await _service.StopAxisAsync(address);
            _uniformRunningAxis = null;

            _state.WriteLog(
                LogLevel.Warning,
                $"Driver {address.DisplayId}: đã Quick Stop test 16 PR. " +
                "Phải HOME lại trước lần START tiếp theo.");
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Error,
                $"[UNIFORM STOP] {address.DisplayId}: {ex.Message}");
        }
    }

    private async Task StopMoveAsync()
    {
        _moveCts?.Cancel();
        await _service.StopAxisAsync(_state.SelectedAxis);
    }

    private void UpdateProgress()
    {
        if (_moveCts is null || _estimatedSeconds <= 0)
        {
            return;
        }

        var elapsed = (DateTime.UtcNow - _moveStarted).TotalSeconds;
        var percent = (int)Math.Clamp(
            elapsed / Math.Max(0.2, _estimatedSeconds) * 100,
            0,
            99);

        _moveProgress.Value = Math.Clamp(
            percent,
            _moveProgress.Minimum,
            _moveProgress.Maximum);
    }

    private void ParseAndSelectAxis()
    {
        if (!AxisAddress.TryParse(_axisInput.Text, out var axis))
        {
            _state.WriteLog(
                LogLevel.Error,
                "ID không hợp lệ. Dùng định dạng 1.1 đến 4.16.");
            _axisInput.BackColor = Color.FromArgb(70, 32, 34);
            return;
        }

        _axisInput.BackColor = UiTheme.SurfaceAlt;
        SelectAxis(axis);
    }

    private void SelectAxis(AxisAddress axis)
    {
        _state.SelectedAxis = axis;
        _axisGrid.SelectedAxis = axis;
        _axisInput.Text = axis.DisplayId;
        _selectedId.Text = axis.DisplayId;

        var line = _state.Lines[axis.Line - 1];
        _selectedInfo.Text =
            $"Line {axis.Line} · Slave {axis.SlaveId} · {line.PortName}\r\n" +
            _state.GetAxis(axis).State.ToString().ToUpperInvariant();

        var statusDriver = FindControlRecursive(this, "StatusDriver") as Label;
        if (statusDriver is not null)
        {
            statusDriver.Text = axis.DisplayId;
        }

        UpdateAxisStatus();
    }

    private void UpdateMoveCalculation()
    {
        var signedRevolutions =
            (double)_revolutions.Value *
            (_direction.SelectedIndex == 0 ? 1 : -1);

        var pulsesLong =
            (long)Math.Round(signedRevolutions * (double)_pulsePerRev.Value);
        var clampedPulses = Math.Clamp(pulsesLong, int.MinValue, int.MaxValue);
        var unsigned = (uint)(int)clampedPulses;
        var high = (ushort)(unsigned >> 16);
        var low = (ushort)(unsigned & 0xFFFF);
        var seconds =
            Math.Abs(signedRevolutions) /
            Math.Max(1, (double)_moveSpeed.Value) *
            60;

        _pulseValue.Text = $"{pulsesLong:N0} p";
        _highValue.Text = $"0x{high:X4}";
        _lowValue.Text = $"0x{low:X4}";
        _timeValue.Text = $"{seconds:0.00} s";
    }

    private void OnStateChanged(object? sender, EventArgs e)
    {
        if (IsDisposed || Disposing || !IsHandleCreated)
        {
            return;
        }

        if (InvokeRequired)
        {
            try
            {
                BeginInvoke(new Action(UpdateAxisStatus));
            }
            catch (InvalidOperationException)
            {
                // Bao gồm cả ObjectDisposedException vì lớp này kế thừa
                // InvalidOperationException. Control đang đóng hoặc đã dispose.
            }

            return;
        }

        UpdateAxisStatus();
    }

    private void UpdateAxisStatus()
    {
        if (IsDisposed || Disposing)
        {
            return;
        }

        var axis = _state.GetAxis(_state.SelectedAxis);
        _selectedInfo.Text =
            $"Line {axis.Address.Line} · Slave {axis.Address.SlaveId} · " +
            $"{_state.Lines[axis.Address.Line - 1].PortName}\r\n" +
            axis.State.ToString().ToUpperInvariant();

        _modeValue.Text =
            axis.LastCommand.StartsWith(
                "MANUAL_UNIFORM_PR_LOOP",
                StringComparison.Ordinal)
                ? "UNIFORM PR"
                : axis.State switch
                {
                    AxisMotionState.JoggingForward or
                    AxisMotionState.JoggingReverse => "JOG",
                    AxisMotionState.Moving => "POSITION",
                    AxisMotionState.Homing => "HOMING",
                    _ => "IDLE"
                };

        _speedValue.Text = $"{axis.VelocityRpm} rpm";
        _commandValue.Text = axis.LastCommand;

        var jogCenter = FindControlRecursive(this, "JogCenterLabel") as Label;
        if (jogCenter is null)
        {
            return;
        }

        jogCenter.Text = axis.State switch
        {
            AxisMotionState.JoggingForward => "MOTOR\r\nCW ▶",
            AxisMotionState.JoggingReverse => "MOTOR\r\n◀ CCW",
            _ => "MOTOR\r\nSTOPPED"
        };

        jogCenter.ForeColor =
            axis.State is AxisMotionState.JoggingForward or AxisMotionState.JoggingReverse
                ? UiTheme.Warning
                : UiTheme.Muted;
    }

    private static Label CreateCalculationLabel(string text)
    {
        var label = UiTheme.Label(
            text,
            new Font("Consolas", 10F, FontStyle.Bold));

        label.AutoSize = false;
        label.AutoEllipsis = true;
        label.TextAlign = ContentAlignment.MiddleLeft;
        label.Dock = DockStyle.Fill;
        label.Margin = new Padding(0);

        return label;
    }

    private static Control BuildHeader(string title, string subtitle)
    {
        // Dùng TableLayoutPanel thay cho Location tuyệt đối. Cách này không bị
        // chồng chữ khi Windows Scale là 125%, 150% hoặc font bị phóng lớn.
        var panel = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            BackColor = Color.Transparent,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 28F));
        panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var titleLabel = UiTheme.Label(title, UiTheme.FontSection);
        titleLabel.Dock = DockStyle.Fill;
        titleLabel.AutoSize = false;
        titleLabel.TextAlign = ContentAlignment.MiddleLeft;
        titleLabel.Margin = new Padding(0);

        var subtitleLabel = UiTheme.Label(
            subtitle,
            UiTheme.FontSmall,
            UiTheme.Muted);
        subtitleLabel.Dock = DockStyle.Fill;
        subtitleLabel.AutoSize = false;
        subtitleLabel.AutoEllipsis = true;
        subtitleLabel.TextAlign = ContentAlignment.TopLeft;
        subtitleLabel.Margin = new Padding(0);

        panel.Controls.Add(titleLabel, 0, 0);
        panel.Controls.Add(subtitleLabel, 0, 1);

        return panel;
    }

    private static Control LabeledControl(string label, Control control)
    {
        var panel = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 2,
            ColumnCount = 1,
            Margin = new Padding(4, 0, 4, 0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 22F));
        panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var title = UiTheme.Label(label, UiTheme.FontSmall, UiTheme.Muted);
        title.Dock = DockStyle.Fill;
        title.AutoSize = false;
        title.AutoEllipsis = true;
        title.TextAlign = ContentAlignment.MiddleLeft;
        title.Margin = new Padding(0);

        control.Dock = DockStyle.Fill;
        control.Margin = new Padding(0);
        control.MinimumSize = new Size(0, 30);

        panel.Controls.Add(title, 0, 0);
        panel.Controls.Add(control, 0, 1);

        return panel;
    }

    private static Control ValueCard(string title, Label valueControl)
    {
        var panel = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 2,
            ColumnCount = 1,
            BackColor = UiTheme.SurfaceAlt,
            Margin = new Padding(3),
            Padding = new Padding(7, 4, 7, 4),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 22F));
        panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var label = UiTheme.Label(title, UiTheme.FontSmall, UiTheme.Muted);
        label.Dock = DockStyle.Fill;
        label.AutoSize = false;
        label.AutoEllipsis = true;
        label.TextAlign = ContentAlignment.MiddleLeft;
        label.Margin = new Padding(0);

        valueControl.Dock = DockStyle.Fill;
        valueControl.AutoSize = false;
        valueControl.AutoEllipsis = true;
        valueControl.TextAlign = ContentAlignment.MiddleLeft;
        valueControl.Font = new Font("Consolas", 10F, FontStyle.Bold);
        valueControl.Margin = new Padding(0);

        panel.Controls.Add(label, 0, 0);
        panel.Controls.Add(valueControl, 0, 1);

        return panel;
    }

    private static Control? FindControlRecursive(Control parent, string name)
    {
        foreach (Control child in parent.Controls)
        {
            if (child.Name == name)
            {
                return child;
            }

            var found = FindControlRecursive(child, name);
            if (found is not null)
            {
                return found;
            }
        }

        return null;
    }

    private void ManualPage_Load(object sender, EventArgs e)
    {
    }
}
