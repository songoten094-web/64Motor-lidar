using System.ComponentModel;
using System.Text.Json;
using WaveMotionControl.Models;
using WaveMotionControl.Services;
using WaveMotionControl.State;
using WaveMotionControl.UI.Controls;

namespace WaveMotionControl.UI.Pages;

[DesignerCategory("UserControl")]
public partial class AutoPage : UserControl
{
    private readonly ApplicationState _state;
    private readonly IRs485Service _service;
    private readonly LogView _logView;
    private readonly WavePreviewControl _preview;

    private readonly NumericUpDown _frequency;
    private readonly NumericUpDown _layerOffset;
    private readonly NumericUpDown _rampUp;
    private readonly NumericUpDown _rampDown;
    private readonly NumericUpDown _clusterWidth;
    private readonly NumericUpDown _clusterHeight;
    private readonly TextBox _driverIdBox;
    private readonly ComboBox _clusterCombo;
    private readonly ComboBox _inspectAxis;
    private readonly ComboBox _effectCombo;
    private readonly ComboBox _waveDirectionCombo;
    private readonly ComboBox _lidarZoneCombo;
    private readonly Button _lidarEnterButton;
    private readonly Button _lidarExitButton;
    private readonly Label _selectedCellLabel;
    private readonly Label _clusterInfo;
    private readonly Label _selectedClusterInfo;
    private readonly Label _onlineInfo;
    private readonly Label _autoState;
    private readonly Label _inspectValue;
    private readonly Label _readinessInfo;
    private readonly Label _speedInfo;

    private readonly Button[,] _gridButtons = new Button[16, 16];
    private readonly List<ClusterDraft> _clusters = new();
    private int _nextClusterId = 1;
    private int _selectedClusterId;
    private (int Row, int Column)? _selectedCell;
    private bool _autoRunning;
    private bool _paused;
    private CancellationTokenSource? _autoStartCts;
    private Task? _autoStartTask;
    private bool _autoStopInProgress;
    private readonly Dictionary<int, int?> _lidarActiveZones = new();
    private readonly Dictionary<int, CancellationTokenSource> _lidarUiWindowCts = new();

    // Tự động lưu cấu hình các cụm AUTO vào LocalAppData.
    private readonly string _clusterConfigFilePath = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
        "WaveMotionControl",
        "auto-clusters.json");
    private bool _loadingClusterConfig;

    private sealed class AutoClusterConfigFile
    {
        public int Version { get; set; } = 1;
        public int SelectedClusterId { get; set; }
        public int NextClusterId { get; set; } = 1;
        public List<AutoClusterConfigItem> Clusters { get; set; } = new();
    }

    private sealed class AutoClusterConfigItem
    {
        public int Id { get; set; }
        public int TopRow { get; set; }
        public int LeftColumn { get; set; }
        public int Width { get; set; }
        public int Height { get; set; }
        public AutoEffectType Effect { get; set; }
        public AutoWaveDirection WaveDirection { get; set; }
        public double LayerOffsetRevolutions { get; set; }
        public double FrequencyHz { get; set; }
        public int LidarRandomSeed { get; set; }
        public List<AutoClusterDriverBinding> Drivers { get; set; } = new();
    }

    private sealed class AutoClusterDriverBinding
    {
        public int Row { get; set; }
        public int Column { get; set; }
        public int Line { get; set; }
        public int SlaveId { get; set; }
    }

    private sealed class DesignerDependencies
    {
        public DesignerDependencies()
        {
            State = new ApplicationState();
            Service = new DemoRs485Service(State);
        }

        public ApplicationState State { get; }
        public IRs485Service Service { get; }
    }

    private sealed class ClusterDraft
    {
        public int Id { get; init; }
        public int TopRow { get; set; }
        public int LeftColumn { get; set; }
        public int Width { get; init; }
        public int Height { get; init; }
        public AutoEffectType Effect { get; set; } = AutoEffectType.WaveFromCenter;
        public AutoWaveDirection WaveDirection { get; set; } = AutoWaveDirection.LeftToRight;
        public double LayerOffsetRevolutions { get; set; } = 0.125;
        public double FrequencyHz { get; set; } = 0.20;
        public int LidarRandomSeed { get; set; } = Random.Shared.Next(1, int.MaxValue);
        public Dictionary<(int Row, int Column), AxisAddress?> Drivers { get; } = new();

        public bool Contains(int row, int column) =>
            row >= TopRow && row < TopRow + Height &&
            column >= LeftColumn && column < LeftColumn + Width;

        public IEnumerable<(int Row, int Column)> Cells()
        {
            for (var r = TopRow; r < TopRow + Height; r++)
                for (var c = LeftColumn; c < LeftColumn + Width; c++)
                    yield return (r, c);
        }
    }

    public AutoPage() : this(new DesignerDependencies()) { }

    private AutoPage(DesignerDependencies dependencies) : this(dependencies.State, dependencies.Service) { }

    public AutoPage(ApplicationState state, IRs485Service service)
    {
        _state = state ?? throw new ArgumentNullException(nameof(state));
        _service = service ?? throw new ArgumentNullException(nameof(service));
        _logView = new LogView(_state);
        _preview = new WavePreviewControl();

        _frequency = UiTheme.Numeric(0.20M, 0.01M, 5, 0.01M, 2);
        _layerOffset = UiTheme.Numeric(0.125M, 0.001M, 1, 0.001M, 3);
        _rampUp = UiTheme.Numeric(0, 0, 120, 0.1M, 1);
        _rampDown = UiTheme.Numeric(0, 0, 120, 0.1M, 1);
        _clusterWidth = UiTheme.Numeric(5, 1, 16, 1);
        _clusterHeight = UiTheme.Numeric(3, 1, 16, 1);
        _driverIdBox = new TextBox { Dock = DockStyle.Fill, Font = new Font("Consolas", 11F, FontStyle.Bold) };
        _clusterCombo = UiTheme.ComboBox();
        _inspectAxis = UiTheme.ComboBox();

        _effectCombo = UiTheme.ComboBox();
        _effectCombo.Items.Add(new EffectItem(AutoEffectType.WaveFromCenter, "Sóng từ tâm — vòng chữ nhật"));
        _effectCombo.Items.Add(new EffectItem(AutoEffectType.WaveHeadToTail, "Sóng từ đầu → cuối"));
        _effectCombo.Items.Add(new EffectItem(AutoEffectType.Lidar, "LIDAR — Random + phản ứng Zone"));
        _effectCombo.SelectedIndex = 0;

        _waveDirectionCombo = UiTheme.ComboBox();
        _waveDirectionCombo.Items.Add(new DirectionItem(AutoWaveDirection.LeftToRight, "Trái → Phải"));
        _waveDirectionCombo.Items.Add(new DirectionItem(AutoWaveDirection.RightToLeft, "Phải → Trái"));
        _waveDirectionCombo.Items.Add(new DirectionItem(AutoWaveDirection.TopToBottom, "Trên → Dưới"));
        _waveDirectionCombo.Items.Add(new DirectionItem(AutoWaveDirection.BottomToTop, "Dưới → Trên"));
        _waveDirectionCombo.SelectedIndex = 0;
        _waveDirectionCombo.Enabled = false;

        _lidarZoneCombo = UiTheme.ComboBox();
        _lidarZoneCombo.Enabled = false;
        _lidarEnterButton = UiTheme.Button("TEST ZONE ENTER", primary: true);
        _lidarExitButton = UiTheme.Button("TEST ZONE EXIT");
        _lidarEnterButton.Enabled = false;
        _lidarExitButton.Enabled = false;

        _selectedCellLabel = NewValueLabel("Chưa chọn ô");
        _clusterInfo = NewValueLabel("0 cụm");
        _selectedClusterInfo = NewValueLabel("Chưa tạo cụm");
        _onlineInfo = NewValueLabel("0 / 64 online");
        _autoState = NewValueLabel("READY");
        _inspectValue = NewValueLabel("—");
        _readinessInfo = NewValueLabel("Chưa tạo cụm");
        _speedInfo = NewValueLabel("12 rpm");

        AutoScaleMode = AutoScaleMode.Dpi;
        BackColor = UiTheme.Background;
        DoubleBuffered = true;

        InitializeComponent();

        // AUTO là trang full-screen trong ContentPanel. Ép Dock=Fill ngay tại
        // chính UserControl để tránh phụ thuộc hoàn toàn vào ShellForm/object initializer.
        // Đây là lớp bảo vệ bổ sung cho lỗi trang AUTO bị co về góc trái.
        AutoSize = false;
        Dock = DockStyle.Fill;
        Margin = Padding.Empty;
        MinimumSize = Size.Empty;
        MaximumSize = Size.Empty;

        BuildUi();
        PopulateInspectAxes();
        BindEvents();
        LoadClusterConfiguration();
        RefreshGrid();
    }

    // Giữ handler này để tương thích với AutoPage.Designer.cs cũ
    // nếu Designer vẫn còn dòng: Load += AutoPage_Load;
    private void AutoPage_Load(object? sender, EventArgs e)
    {
        AutoSize = false;
        Dock = DockStyle.Fill;
    }

    private void BuildUi()
    {
        Controls.Clear();

        // Không dùng TableLayoutPanel ở cấp root nữa. Ảnh chạy thực tế cho thấy
        // WinForms có thể co root TableLayout về preferred-size rất nhỏ khi kết hợp
        // DPI scaling + UserControl động. Ở đây ta bố trí ba cột bằng Bounds trực tiếp:
        // LEFT | CENTER GRID | RIGHT. Cách này không phụ thuộc preferred-size.
        var root = new Panel
        {
            Name = "AutoRootPanel",
            Dock = DockStyle.Fill,
            BackColor = UiTheme.Background,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };

        var left = BuildLeftPanel();
        var center = BuildGridPanel();
        var right = BuildRightPanel();

        left.Name = "AutoLeftPanel";
        center.Name = "AutoGridPanel";
        right.Name = "AutoRightPanel";

        // Top-level children được layout thủ công; không dùng Dock ở ba control này.
        left.Dock = DockStyle.None;
        center.Dock = DockStyle.None;
        right.Dock = DockStyle.None;
        left.Margin = Padding.Empty;
        center.Margin = Padding.Empty;
        right.Margin = Padding.Empty;

        root.Controls.Add(left);
        root.Controls.Add(center);
        root.Controls.Add(right);
        Controls.Add(root);

        void ApplyRootLayout()
        {
            if (root.IsDisposed) return;

            var width = root.ClientSize.Width;
            var height = root.ClientSize.Height;
            if (width <= 0 || height <= 0) return;

            const int outer = 10;
            const int gap = 8;

            // Sidebar giữ đủ rộng để không chồng control, nhưng tự co nhẹ trên màn nhỏ.
            var sideWidth = width >= 1500 ? 310 : width >= 1250 ? 295 : 270;
            var usableHeight = Math.Max(1, height - outer * 2);

            // Nếu viewport quá nhỏ, ưu tiên giữ Grid còn nhìn được thay vì tạo width âm.
            var maxSide = Math.Max(220, (width - outer * 2 - gap * 2 - 360) / 2);
            sideWidth = Math.Min(sideWidth, maxSide);

            var leftX = outer;
            var rightX = width - outer - sideWidth;
            var centerX = leftX + sideWidth + gap;
            var centerWidth = Math.Max(1, rightX - gap - centerX);

            left.SetBounds(leftX, outer, sideWidth, usableHeight);
            center.SetBounds(centerX, outer, centerWidth, usableHeight);
            right.SetBounds(rightX, outer, sideWidth, usableHeight);
        }

        root.Resize += (_, _) => ApplyRootLayout();
        root.HandleCreated += (_, _) => ApplyRootLayout();
        VisibleChanged += (_, _) =>
        {
            if (Visible)
            {
                Dock = DockStyle.Fill;
                ApplyRootLayout();
            }
        };
        ParentChanged += (_, _) =>
        {
            if (Parent is not null)
            {
                Dock = DockStyle.Fill;
                ApplyRootLayout();
            }
        };

        ApplyRootLayout();
    }

    private Control BuildLeftPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0, 0, 8, 0);

        // IMPORTANT: keep the scroll viewport Dock=Fill, but never combine
        // AutoScroll with an AutoSize TableLayoutPanel. On some DPI/layout
        // combinations WinForms collapses the preferred width of the page.
        var scroll = new Panel
        {
            Dock = DockStyle.Fill,
            AutoScroll = true,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = false,
            Height = 1220,
            ColumnCount = 1,
            RowCount = 6,
            Margin = Padding.Empty,
            Padding = Padding.Empty,
            BackColor = Color.Transparent,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 190));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 125));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 175));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 450));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 210));

        var header = BuildHeader("AUTO — CỤM CHUYỂN ĐỘNG", "Grid 16×16 · nhiều cụm độc lập");
        header.Dock = DockStyle.Fill;
        header.Margin = new Padding(0, 0, 0, 8);
        layout.Controls.Add(header, 0, 0);

        var create = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 4,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 4, 0, 8),
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        create.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        create.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        create.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
        create.RowStyles.Add(new RowStyle(SizeType.Absolute, 54));
        create.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        create.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var createTitle = UiTheme.Label("Tạo cụm mới", UiTheme.FontSection, UiTheme.Accent);
        createTitle.Dock = DockStyle.Fill;
        createTitle.TextAlign = ContentAlignment.MiddleLeft;
        create.Controls.Add(createTitle, 0, 0);
        create.SetColumnSpan(createTitle, 2);
        create.Controls.Add(LabeledControl("Dài (cột)", _clusterWidth), 0, 1);
        create.Controls.Add(LabeledControl("Rộng (hàng)", _clusterHeight), 1, 1);
        var add = UiTheme.Button("+ TẠO CỤM", primary: true);
        add.Dock = DockStyle.Fill;
        add.Click += (_, _) => CreateCluster();
        create.Controls.Add(add, 0, 2);
        var delete = UiTheme.Button("XÓA CỤM");
        delete.Dock = DockStyle.Fill;
        delete.Click += (_, _) => DeleteSelectedCluster();
        create.Controls.Add(delete, 1, 2);
        var createHelp = UiTheme.Label(
            "Tạo cụm → chọn cụm → click Grid để đặt góc trên-trái.",
            UiTheme.FontSmall,
            UiTheme.Muted);
        createHelp.Dock = DockStyle.Fill;
        createHelp.AutoSize = false;
        createHelp.TextAlign = ContentAlignment.TopLeft;
        create.Controls.Add(createHelp, 0, 3);
        create.SetColumnSpan(createHelp, 2);
        layout.Controls.Add(create, 0, 1);

        var clusterBox = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 3,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 4, 0, 8),
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        clusterBox.RowStyles.Add(new RowStyle(SizeType.Absolute, 32));
        clusterBox.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        clusterBox.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var clusterTitle = UiTheme.Label("CỤM ĐANG CHỌN", UiTheme.FontSection, UiTheme.Muted);
        clusterTitle.Dock = DockStyle.Fill;
        clusterTitle.TextAlign = ContentAlignment.MiddleLeft;
        clusterBox.Controls.Add(clusterTitle, 0, 0);
        _clusterCombo.Dock = DockStyle.Fill;
        clusterBox.Controls.Add(_clusterCombo, 0, 1);
        _selectedClusterInfo.Dock = DockStyle.Fill;
        _selectedClusterInfo.AutoSize = false;
        _selectedClusterInfo.TextAlign = ContentAlignment.MiddleLeft;
        clusterBox.Controls.Add(_selectedClusterInfo, 0, 2);
        layout.Controls.Add(clusterBox, 0, 2);

        var idBox = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 4,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 4, 0, 8),
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        idBox.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
        idBox.RowStyles.Add(new RowStyle(SizeType.Absolute, 42));
        idBox.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        idBox.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var idTitle = UiTheme.Label("GÁN DRIVER ID CHO Ô", UiTheme.FontSection, UiTheme.Muted);
        idTitle.Dock = DockStyle.Fill;
        idTitle.TextAlign = ContentAlignment.MiddleLeft;
        idBox.Controls.Add(idTitle, 0, 0);
        _selectedCellLabel.Dock = DockStyle.Fill;
        _selectedCellLabel.AutoSize = false;
        _selectedCellLabel.TextAlign = ContentAlignment.MiddleLeft;
        idBox.Controls.Add(_selectedCellLabel, 0, 1);
        var idRow = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };
        idRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 62));
        idRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 38));
        _driverIdBox.Dock = DockStyle.Fill;
        idRow.Controls.Add(_driverIdBox, 0, 0);
        var setId = UiTheme.Button("GÁN ID");
        setId.Dock = DockStyle.Fill;
        setId.Click += (_, _) => AssignDriverId();
        idRow.Controls.Add(setId, 1, 0);
        idBox.Controls.Add(idRow, 0, 2);
        var clearId = UiTheme.Button("XÓA ID Ô ĐANG CHỌN");
        clearId.Dock = DockStyle.Fill;
        clearId.Click += (_, _) => ClearSelectedCell();
        idBox.Controls.Add(clearId, 0, 3);
        layout.Controls.Add(idBox, 0, 3);

        var effect = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 8,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 4, 0, 8),
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        effect.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
        effect.RowStyles.Add(new RowStyle(SizeType.Absolute, 50));
        effect.RowStyles.Add(new RowStyle(SizeType.Absolute, 50));
        effect.RowStyles.Add(new RowStyle(SizeType.Absolute, 54));
        effect.RowStyles.Add(new RowStyle(SizeType.Absolute, 62));
        effect.RowStyles.Add(new RowStyle(SizeType.Absolute, 64));
        effect.RowStyles.Add(new RowStyle(SizeType.Absolute, 58));
        effect.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var effectTitle = UiTheme.Label("HIỆU ỨNG CỤM", UiTheme.FontSection, UiTheme.Muted);
        effectTitle.Dock = DockStyle.Fill;
        effectTitle.TextAlign = ContentAlignment.MiddleLeft;
        effect.Controls.Add(effectTitle, 0, 0);
        effect.Controls.Add(LabeledControl("Hiệu ứng", _effectCombo), 0, 1);
        effect.Controls.Add(LabeledControl("Hướng sóng (Đầu → cuối)", _waveDirectionCombo), 0, 2);
        effect.Controls.Add(LabeledControl("Lệch lớp (vòng)", _layerOffset), 0, 3);
        effect.Controls.Add(LabeledControl("Tốc độ motor (vòng/s)", _frequency), 0, 4);
        effect.Controls.Add(ValueCard("Tốc độ tương đương", _speedInfo), 0, 5);
        effect.Controls.Add(LabeledControl("Mô phỏng LIDAR: 1 Zone = 1 cột", _lidarZoneCombo), 0, 6);

        var lidarTestButtons = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };
        lidarTestButtons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        lidarTestButtons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        _lidarEnterButton.Dock = DockStyle.Fill;
        _lidarExitButton.Dock = DockStyle.Fill;
        _lidarEnterButton.Click += async (_, _) => await SimulateLidarZoneEnterAsync();
        _lidarExitButton.Click += async (_, _) => await SimulateLidarZoneExitAsync();
        lidarTestButtons.Controls.Add(_lidarEnterButton, 0, 0);
        lidarTestButtons.Controls.Add(_lidarExitButton, 1, 0);
        effect.Controls.Add(lidarTestButtons, 0, 7);
        layout.Controls.Add(effect, 0, 4);

        var help = UiTheme.Label(
            "Điều kiện AUTO START:\n" +
            "• Tất cả ô trong cụm phải có Driver ID.\n" +
            "• 100% driver của cụm phải Online.\n" +
            "• Mỗi driver phải HOME hoặc đã lấy vị trí hiện tại làm gốc.\n" +
            "• Hiện tại motor quay đều một chiều; chưa bù tốc độ slider-crank.\n" +
            "• Sóng từ tâm: lan theo các vòng chữ nhật đồng tâm.\n" +
            "• Sóng từ đầu → cuối: quét theo hàng/cột theo hướng đã chọn.\n" +
            "• LIDAR: nền random cùng tốc độ/khác pha; Zone active re-phase cột tâm = 0,5 vòng, hai bên giảm dần rồi tiếp tục quay cùng tốc độ.\n" +
            "• Hiện tại nút TEST Zone chỉ mô phỏng tín hiệu; CHƯA nối cảm biến LiDAR thật.\n\n" +
            "Pha 0 của HOME: con trượt ở phía trong.",
            UiTheme.FontSmall,
            UiTheme.Muted);
        help.Dock = DockStyle.Fill;
        help.AutoSize = false;
        help.TextAlign = ContentAlignment.TopLeft;
        help.Padding = new Padding(2, 8, 2, 4);
        layout.Controls.Add(help, 0, 5);

        scroll.Controls.Add(layout);
        card.Controls.Add(scroll);
        return card;
    }

    private Control BuildGridPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;
        card.Margin = new Padding(0, 0, 8, 0);
        var layout = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 2, ColumnCount = 1, Padding = new Padding(10), BackColor = Color.Transparent };
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 62));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        layout.Controls.Add(BuildHeader("BẢN ĐỒ CƠ CẤU 16 × 16", "Mỗi ô chứa tối đa một Driver ID · tâm hình học dùng để tạo sóng"), 0, 0);

        var grid = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 16, ColumnCount = 16, BackColor = Color.FromArgb(18, 24, 35), Margin = new Padding(8) };
        for (var i = 0; i < 16; i++)
        {
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 6.25f));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 6.25f));
        }
        for (var r = 0; r < 16; r++)
            for (var c = 0; c < 16; c++)
            {
                var rr = r; var cc = c;
                var button = new Button
                {
                    Dock = DockStyle.Fill,
                    Margin = new Padding(1),
                    FlatStyle = FlatStyle.Flat,
                    FlatAppearance = { BorderColor = Color.FromArgb(50, 65, 85), BorderSize = 1 },
                    BackColor = Color.FromArgb(22, 30, 43),
                    ForeColor = Color.White,
                    Font = new Font("Segoe UI", 8.5F, FontStyle.Bold),
                    Text = "",
                    Tag = (rr, cc)
                };
                button.Click += (_, _) => SelectGridCell(rr, cc);
                _gridButtons[r, c] = button;
                grid.Controls.Add(button, c, r);
            }
        layout.Controls.Add(grid, 0, 1);
        card.Controls.Add(layout);
        return card;
    }

    private Control BuildRightPanel()
    {
        var card = UiTheme.Card();
        card.Dock = DockStyle.Fill;

        var scroll = new Panel
        {
            Dock = DockStyle.Fill,
            AutoScroll = true,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = false,
            Height = 1210,
            RowCount = 7,
            ColumnCount = 1,
            Margin = Padding.Empty,
            Padding = Padding.Empty,
            BackColor = Color.Transparent,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 70));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 255));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 190));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 320));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 200));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 110));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 65));

        var header = BuildHeader("AUTO STATUS", "Mỗi cụm có tốc độ riêng · START chung");
        header.Dock = DockStyle.Fill;
        header.Margin = new Padding(0, 0, 0, 8);
        layout.Controls.Add(header, 0, 0);

        var status = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 4,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 4, 0, 8),
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        for (var i = 0; i < 4; i++) status.RowStyles.Add(new RowStyle(SizeType.Percent, 25));
        status.Controls.Add(ValueCard("Trạng thái", _autoState), 0, 0);
        status.Controls.Add(ValueCard("Điều kiện START", _readinessInfo), 0, 1);
        status.Controls.Add(ValueCard("Driver online toàn hệ thống", _onlineInfo), 0, 2);
        status.Controls.Add(ValueCard("Số cụm", _clusterInfo), 0, 3);
        layout.Controls.Add(status, 0, 1);

        var inspect = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 4,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 4, 0, 8),
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        inspect.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
        inspect.RowStyles.Add(new RowStyle(SizeType.Absolute, 54));
        inspect.RowStyles.Add(new RowStyle(SizeType.Absolute, 62));
        inspect.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var inspectTitle = UiTheme.Label("KIỂM TRA DRIVER", UiTheme.FontSection, UiTheme.Muted);
        inspectTitle.Dock = DockStyle.Fill;
        inspectTitle.TextAlign = ContentAlignment.MiddleLeft;
        inspect.Controls.Add(inspectTitle, 0, 0);
        inspect.Controls.Add(LabeledControl("ID", _inspectAxis), 0, 1);
        inspect.Controls.Add(ValueCard("Pha / vị trí", _inspectValue), 0, 2);
        var inspectHelp = UiTheme.Label(
            "AUTO 16PR nội bộ: các layer được đặt sẵn lệch pha, sau đó cùng chạy; không delay layer bằng PC.",
            UiTheme.FontSmall,
            UiTheme.Muted);
        inspectHelp.Dock = DockStyle.Fill;
        inspectHelp.AutoSize = false;
        inspectHelp.TextAlign = ContentAlignment.TopLeft;
        inspect.Controls.Add(inspectHelp, 0, 3);
        layout.Controls.Add(inspect, 0, 2);

        var buttons = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 6,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 4, 0, 8),
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
        for (var i = 0; i < 6; i++) buttons.RowStyles.Add(new RowStyle(SizeType.Percent, 16.6667F));

        var preview = UiTheme.Button("CHẠY / DỪNG PREVIEW", primary: true);
        preview.Dock = DockStyle.Fill;
        preview.Click += (_, _) =>
        {
            _preview.Running = !_preview.Running;
            preview.Text = _preview.Running ? "DỪNG PREVIEW" : "CHẠY / DỪNG PREVIEW";
            _state.WriteLog(LogLevel.Info, _preview.Running ? "AUTO Preview bắt đầu." : "AUTO Preview dừng.");
        };
        var origin = UiTheme.Button("LẤY VỊ TRÍ HIỆN TẠI = GỐC");
        origin.Dock = DockStyle.Fill;
        origin.Click += async (_, _) => await SetSelectedClusterOriginAsync();
        var startButton = UiTheme.Button("AUTO START", primary: true);
        startButton.Dock = DockStyle.Fill;
        startButton.Click += async (_, _) => await StartAutoAsync();
        var pause = UiTheme.Button("PAUSE / RESUME");
        pause.Dock = DockStyle.Fill;
        pause.Click += async (_, _) => await TogglePauseAsync();
        var stop = UiTheme.Button("STOP");
        stop.Dock = DockStyle.Fill;
        stop.Click += async (_, _) => await StopAutoAsync(false);
        var quickStop = UiTheme.Button("QUICK STOP", danger: true);
        quickStop.Dock = DockStyle.Fill;
        quickStop.Click += async (_, _) => await StopAutoAsync(true);

        buttons.Controls.Add(preview, 0, 0);
        buttons.Controls.Add(origin, 0, 1);
        buttons.Controls.Add(startButton, 0, 2);
        buttons.Controls.Add(pause, 0, 3);
        buttons.Controls.Add(stop, 0, 4);
        buttons.Controls.Add(quickStop, 0, 5);
        layout.Controls.Add(buttons, 0, 3);

        _logView.Dock = DockStyle.Fill;
        _logView.Margin = new Padding(0, 4, 0, 8);
        layout.Controls.Add(_logView, 0, 4);

        var footer = UiTheme.Label(
            "AUTO: quay motor đều một chiều. Mỗi cụm có hiệu ứng, hướng sóng và tốc độ riêng. " +
            "HOME = pha 0 với con trượt ở phía trong.",
            UiTheme.FontSmall,
            UiTheme.Accent);
        footer.Dock = DockStyle.Fill;
        footer.AutoSize = false;
        footer.TextAlign = ContentAlignment.TopLeft;
        footer.Padding = new Padding(2, 8, 2, 0);
        layout.Controls.Add(footer, 0, 5);

        var safety = UiTheme.Label(
            "START chỉ cho phép khi toàn bộ driver trong tất cả cụm đã sẵn sàng.",
            UiTheme.FontSmall,
            UiTheme.Muted);
        safety.Dock = DockStyle.Fill;
        safety.AutoSize = false;
        safety.TextAlign = ContentAlignment.TopLeft;
        layout.Controls.Add(safety, 0, 6);

        scroll.Controls.Add(layout);
        card.Controls.Add(scroll);
        return card;
    }

    private void BindEvents()
    {
        _clusterCombo.SelectedIndexChanged += (_, _) =>
        {
            if (_clusterCombo.SelectedItem is ClusterItem item)
            {
                _selectedClusterId = item.Id;
                LoadSelectedClusterSettings();
                RefreshGrid();
                RefreshLidarSimulationControls();
                SaveClusterConfiguration();
            }
        };
        _effectCombo.SelectedIndexChanged += (_, _) =>
        {
            if (SelectedCluster() is { } c)
                c.Effect = SelectedEffect();

            var effect = SelectedEffect();
            _waveDirectionCombo.Enabled = effect == AutoEffectType.WaveHeadToTail;
            _layerOffset.Enabled = effect != AutoEffectType.Lidar;
            if (SelectedCluster() is { } selected && effect == AutoEffectType.Lidar && selected.LidarRandomSeed == 0)
                selected.LidarRandomSeed = Random.Shared.Next(1, int.MaxValue);
            RefreshLidarSimulationControls();
            RefreshGrid();
            SaveClusterConfiguration();
        };
        _waveDirectionCombo.SelectedIndexChanged += (_, _) =>
        {
            if (SelectedCluster() is { } c)
                c.WaveDirection = SelectedWaveDirection();

            RefreshGrid();
            SaveClusterConfiguration();
        };

        _layerOffset.ValueChanged += (_, _) =>
        {
            if (SelectedCluster() is { } c) c.LayerOffsetRevolutions = (double)_layerOffset.Value;
            _preview.Invalidate();
            SaveClusterConfiguration();
        };
        _frequency.ValueChanged += (_, _) =>
        {
            if (SelectedCluster() is { } c) c.FrequencyHz = (double)_frequency.Value;
            RefreshSpeedInfo();
            RefreshAutoReadiness();
            _preview.Invalidate();
            SaveClusterConfiguration();
        };
        _inspectAxis.SelectedIndexChanged += (_, _) => _preview.Invalidate();
        _state.StateChanged += (_, _) => BeginInvokeSafe(RefreshOnlineInfo);
        _preview.ProgramProvider = TryBuildProgram;
        _preview.LidarZoneProvider = clusterId =>
            _lidarActiveZones.TryGetValue(clusterId, out var zone) ? zone : null;
        _preview.InspectAxis = new AxisAddress(1, 1);
        var inspectTimer = new System.Windows.Forms.Timer { Interval = 100 };
        inspectTimer.Tick += (_, _) => UpdateInspectValue();
        inspectTimer.Start();
        var gridTimer = new System.Windows.Forms.Timer { Interval = 250 };
        gridTimer.Tick += (_, _) => RefreshGrid();
        gridTimer.Start();
        Disposed += (_, _) =>
        {
            SaveClusterConfiguration();
            CancelLidarUiWindowTimers();
            inspectTimer.Dispose();
            gridTimer.Dispose();
        };
    }

    private void PopulateInspectAxes()
    {
        _inspectAxis.Items.Clear();
        _inspectAxis.Items.AddRange(AxisAddress.All().Select(a => a.DisplayId).Cast<object>().ToArray());
        _inspectAxis.SelectedIndex = 0;
    }

    private void CreateCluster()
    {
        var width = (int)_clusterWidth.Value;
        var height = (int)_clusterHeight.Value;
        var position = FindFirstFreePosition(width, height);
        if (position is null)
        {
            MessageBox.Show(this, "Không còn vùng trống phù hợp trong Grid 16×16.", "Tạo cụm", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        var draft = new ClusterDraft
        {
            Id = GetNextAvailableClusterId(),
            TopRow = position.Value.Row,
            LeftColumn = position.Value.Column,
            Width = width,
            Height = height,
            Effect = SelectedEffect(),
            WaveDirection = SelectedWaveDirection(),
            LayerOffsetRevolutions = (double)_layerOffset.Value,
            FrequencyHz = (double)_frequency.Value,
            LidarRandomSeed = Random.Shared.Next(1, int.MaxValue)
        };
        foreach (var cell in draft.Cells()) draft.Drivers[cell] = null;
        _clusters.Add(draft);
        _selectedClusterId = draft.Id;
        RebuildClusterCombo();
        _state.WriteLog(LogLevel.Ok, $"Đã tạo Cụm {draft.Id}: {width}×{height}. Click Grid để đặt góc trên-trái, sau đó gán ID từng ô.");
        RefreshGrid();
        SaveClusterConfiguration();
    }

    private void DeleteSelectedCluster()
    {
        var cluster = SelectedCluster();

        if (cluster is null)
            return;

        // Không xóa cấu trúc khi AUTO đang chạy.
        if (_autoRunning)
        {
            MessageBox.Show(
                this,
                "Hãy STOP AUTO trước khi xóa cụm.",
                "Xóa cụm",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning);

            return;
        }

        var answer = MessageBox.Show(
            this,
            $"Bạn có chắc muốn xóa Cụm {cluster.Id}?\r\n\r\n" +
            $"Cụm này đang tương ứng với Surface S{cluster.Id:00}.",
            "Xác nhận xóa cụm",
            MessageBoxButtons.YesNo,
            MessageBoxIcon.Warning);

        if (answer != DialogResult.Yes)
            return;

        _clusters.Remove(cluster);

        _selectedClusterId =
            _clusters
                .OrderBy(c => c.Id)
                .FirstOrDefault()?.Id ?? 0;

        _selectedCell = null;

        RebuildClusterCombo();
        RefreshGrid();

        _state.WriteLog(
            LogLevel.Warning,
            $"Đã xóa Cụm {cluster.Id}. " +
            $"Surface S{cluster.Id:00} hiện chưa có cụm motor tương ứng.");
    }

    private void RebuildClusterCombo()
    {
        _clusterCombo.Items.Clear();
        foreach (var cluster in _clusters.OrderBy(c => c.Id)) _clusterCombo.Items.Add(new ClusterItem(cluster.Id, $"Cụm {cluster.Id} — {cluster.Width}×{cluster.Height}"));
        if (_selectedClusterId != 0)
        {
            var index = _clusterCombo.Items.Cast<ClusterItem>().ToList().FindIndex(x => x.Id == _selectedClusterId);
            if (index >= 0) _clusterCombo.SelectedIndex = index;
        }
        else if (_clusterCombo.Items.Count > 0) _clusterCombo.SelectedIndex = 0;
        _clusterInfo.Text = $"{_clusters.Count} cụm";
        var selected = SelectedCluster();
        _selectedClusterInfo.Text = selected is null
            ? "Chưa tạo cụm"
            : $"Cụm {selected.Id}: {selected.Width}×{selected.Height} · {EffectSummary(selected)} · {selected.FrequencyHz:0.00} vòng/s";
    }

    private void SelectGridCell(int row, int column)
    {
        var cluster = SelectedCluster();
        if (cluster is null)
        {
            _selectedCell = (row, column);
            _selectedCellLabel.Text = $"Ô [{row + 1},{column + 1}] — chưa có cụm";
            return;
        }

        if (!cluster.Contains(row, column))
        {
            if (!MoveClusterTo(cluster, row, column)) return;
            _selectedCell = (row, column);
            _selectedCellLabel.Text = $"Cụm {cluster.Id} · góc trên-trái [{row + 1},{column + 1}]";
            _driverIdBox.Clear();
            _state.WriteLog(LogLevel.Info, $"Cụm {cluster.Id} đặt tại hàng {row + 1}, cột {column + 1}.");
        }
        else
        {
            _selectedCell = (row, column);
            _selectedCellLabel.Text = $"Cụm {cluster.Id} · ô [{row + 1},{column + 1}]";
            _driverIdBox.Text = cluster.Drivers.TryGetValue((row, column), out var driver) && driver is AxisAddress a ? a.DisplayId : string.Empty;
        }
        RefreshGrid();
    }

    private bool MoveClusterTo(ClusterDraft cluster, int topRow, int leftColumn)
    {
        if (topRow + cluster.Height > 16 || leftColumn + cluster.Width > 16)
        {
            MessageBox.Show(this, "Cụm không được vượt khỏi Grid 16×16.", "Vị trí cụm", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return false;
        }

        if (_clusters.Any(other => other != cluster && RectsOverlap(topRow, leftColumn, cluster.Height, cluster.Width, other.TopRow, other.LeftColumn, other.Height, other.Width)))
        {
            MessageBox.Show(this, "Vị trí mới bị chồng lên cụm khác.", "Vị trí cụm", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return false;
        }

        var oldTop = cluster.TopRow;
        var oldLeft = cluster.LeftColumn;
        var old = cluster.Drivers.ToArray();
        cluster.TopRow = topRow;
        cluster.LeftColumn = leftColumn;
        cluster.Drivers.Clear();
        foreach (var cell in cluster.Cells()) cluster.Drivers[cell] = null;
        foreach (var pair in old)
        {
            var localRow = pair.Key.Row - oldTop;
            var localCol = pair.Key.Column - oldLeft;
            if (localRow >= 0 && localRow < cluster.Height && localCol >= 0 && localCol < cluster.Width)
                cluster.Drivers[(cluster.TopRow + localRow, cluster.LeftColumn + localCol)] = pair.Value;
        }
        SaveClusterConfiguration();
        return true;
    }

    private void AssignDriverId()
    {
        var cluster = SelectedCluster();
        if (cluster is null || _selectedCell is null)
        {
            MessageBox.Show(this, "Hãy chọn một cụm và một ô trong cụm.", "Gán ID", MessageBoxButtons.OK, MessageBoxIcon.Information);
            return;
        }

        var cell = _selectedCell.Value;
        if (!cluster.Contains(cell.Row, cell.Column)) return;
        if (!TryParseDriverId(_driverIdBox.Text, out var driver))
        {
            MessageBox.Show(this, "ID phải có dạng 1.1 … 4.16 hoặc số 1 … 64.", "Gán ID", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        var duplicate = _clusters.Any(c => c.Drivers.Any(x => x.Key != cell && x.Value == driver));
        if (duplicate)
        {
            MessageBox.Show(this, $"Driver {driver.DisplayId} đã được gán ở ô khác.", "Gán ID", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        cluster.Drivers[cell] = driver;
        _state.WriteLog(LogLevel.Info, $"Cụm {cluster.Id}: ô [{cell.Row + 1},{cell.Column + 1}] = Driver {driver.DisplayId}.");
        RefreshGrid();
        SaveClusterConfiguration();
    }

    private void ClearSelectedCell()
    {
        var cluster = SelectedCluster();
        if (cluster is null || _selectedCell is null) return;
        var cell = _selectedCell.Value;
        if (cluster.Drivers.ContainsKey(cell)) cluster.Drivers[cell] = null;
        _driverIdBox.Clear();
        RefreshGrid();
        SaveClusterConfiguration();
    }

    private ClusterDraft? SelectedCluster() => _clusters.FirstOrDefault(c => c.Id == _selectedClusterId);

    private void LoadSelectedClusterSettings()
    {
        var cluster = SelectedCluster();
        if (cluster is null) return;
        SelectEffect(cluster.Effect);
        SelectWaveDirection(cluster.WaveDirection);
        _waveDirectionCombo.Enabled = cluster.Effect == AutoEffectType.WaveHeadToTail;
        _layerOffset.Enabled = cluster.Effect != AutoEffectType.Lidar;
        _layerOffset.Value = (decimal)Math.Clamp(cluster.LayerOffsetRevolutions, (double)_layerOffset.Minimum, (double)_layerOffset.Maximum);
        _frequency.Value = (decimal)Math.Clamp(cluster.FrequencyHz, (double)_frequency.Minimum, (double)_frequency.Maximum);
        _clusterWidth.Value = cluster.Width;
        _clusterHeight.Value = cluster.Height;
        RefreshSpeedInfo();
        RefreshAutoReadiness();
        RefreshLidarSimulationControls();
    }

    private void RefreshGrid()
    {
        foreach (var button in _gridButtons)
        {
            button.Text = string.Empty;
            button.BackColor = Color.FromArgb(22, 30, 43);
            button.ForeColor = Color.White;
        }

        foreach (var cluster in _clusters)
        {
            var color = ColorForCluster(cluster.Id);
            var previewLayers = ToAutoCluster(cluster).BuildWaveLayers();
            var maxLayerIndex = previewLayers.Count == 0 ? 0 : previewLayers.Max(layer => layer.Index);
            var layerMap = previewLayers
                .SelectMany(layer => layer.Drivers.Select(driver => (driver, layer.Index)))
                .ToDictionary(x => x.driver, x => x.Index);
            foreach (var cell in cluster.Cells())
            {
                var b = _gridButtons[cell.Row, cell.Column];
                b.BackColor = Color.FromArgb(70, color);
                if (cluster.Drivers.TryGetValue(cell, out var driver) && driver is AxisAddress a)
                {
                    if (_preview.Running && layerMap.TryGetValue(a, out var layerIndex))
                    {
                        var modelCluster = ToAutoCluster(cluster);
                        double phase;
                        if (cluster.Effect == AutoEffectType.Lidar)
                        {
                            var localColumn = cell.Column - cluster.LeftColumn;
                            var multiplier =
                                _lidarActiveZones.TryGetValue(cluster.Id, out var activeZone) &&
                                activeZone is int zone &&
                                localColumn == zone
                                    ? 2.0
                                    : 1.0;

                            phase = modelCluster.GetLidarRandomPhase(a) +
                                    _preview.CurrentTimeSeconds *
                                    Math.Max(0.0001, cluster.FrequencyHz) *
                                    multiplier;
                        }
                        else
                        {
                            var frequency = Math.Max(0.0001, cluster.FrequencyHz);
                            var rawPhaseOffset =
                                (maxLayerIndex - layerIndex) *
                                Math.Max(0, cluster.LayerOffsetRevolutions);
                            var phaseOffset = rawPhaseOffset % 1.0;
                            phase = phaseOffset + _preview.CurrentTimeSeconds * frequency;
                        }

                        b.BackColor = Color.FromArgb(170, ColorFromHsv((float)((phase * 360.0 + layerIndex * 25.0) % 360.0), 0.72f, 0.92f));
                    }
                    b.Text = a.DisplayId;
                    var runtime = _state.GetAxis(a);
                    b.ForeColor = !runtime.IsOnline
                        ? UiTheme.Error
                        : runtime.State != AxisMotionState.Homed
                            ? UiTheme.Warning
                            : Color.White;
                }
                else
                {
                    b.Text = "—";
                    b.ForeColor = UiTheme.Muted;
                }
            }
        }

        if (_selectedCell is { } selected && selected.Row is >= 0 and < 16 && selected.Column is >= 0 and < 16)
            _gridButtons[selected.Row, selected.Column].BackColor = UiTheme.Accent;

        _clusterInfo.Text = $"{_clusters.Count} cụm";
        var selectedCluster = SelectedCluster();
        _selectedClusterInfo.Text = selectedCluster is null
            ? "Chưa tạo cụm"
            : $"Cụm {selectedCluster.Id}: {selectedCluster.Width}×{selectedCluster.Height} · {EffectSummary(selectedCluster)} · {selectedCluster.FrequencyHz:0.00} vòng/s";
        _onlineInfo.Text = $"{_state.OnlineCount} / 64 online";
        RefreshAutoReadiness();
        RefreshSpeedInfo();
        _preview.Invalidate();
    }

    private void RefreshOnlineInfo()
    {
        _onlineInfo.Text = $"{_state.OnlineCount} / 64 online";
        RefreshAutoReadiness();
    }

    private void RefreshSpeedInfo()
    {
        var rps = (double)_frequency.Value;
        _speedInfo.Text = $"{rps:0.00} vòng/s = {rps * 60.0:0.#} rpm";
    }

    private (bool Ready, string Message) GetAutoReadiness()
    {
        if (_clusters.Count == 0)
            return (false, "Chưa tạo cụm");

        foreach (var cluster in _clusters.OrderBy(c => c.Id))
        {
            var missing = cluster.Cells()
                .Where(cell => !cluster.Drivers.TryGetValue(cell, out var driver) || driver is null)
                .ToArray();
            if (missing.Length > 0)
                return (false, $"Cụm {cluster.Id}: thiếu ID {missing.Length} ô");

            var drivers = cluster.Cells()
                .Select(cell => cluster.Drivers[cell]!.Value)
                .Distinct()
                .ToArray();

            var offline = drivers.Where(address => !_state.GetAxis(address).IsOnline).ToArray();
            if (offline.Length > 0)
                return (false, $"Cụm {cluster.Id}: {offline.Length} driver offline");

            var noOrigin = drivers
                .Where(address =>
                {
                    var axis = _state.GetAxis(address);
                    return axis.State != AxisMotionState.Homed ||
                           Math.Abs(axis.PositionRevolutions) > 0.02;
                })
                .ToArray();
            if (noOrigin.Length > 0)
                return (false, $"Cụm {cluster.Id}: {noOrigin.Length} driver chưa ở pha 0");
        }

        var total = _clusters.Sum(c => c.Width * c.Height);
        return (true, $"READY · {total} driver");
    }

    private void RefreshAutoReadiness()
    {
        var readiness = GetAutoReadiness();
        _readinessInfo.Text = readiness.Message;
        _readinessInfo.ForeColor = readiness.Ready ? UiTheme.Online : UiTheme.Warning;
        if (!_autoRunning)
        {
            _autoState.Text = readiness.Ready ? "READY" : "NOT READY";
            _autoState.ForeColor = readiness.Ready ? UiTheme.Online : UiTheme.Warning;
        }
    }

    private (int Row, int Column)? FindFirstFreePosition(int width, int height)
    {
        for (var r = 0; r <= 16 - height; r++)
            for (var c = 0; c <= 16 - width; c++)
            {
                if (_clusters.All(x => !RectsOverlap(r, c, height, width, x.TopRow, x.LeftColumn, x.Height, x.Width))) return (r, c);
            }
        return null;
    }

    private static bool RectsOverlap(int r1, int c1, int h1, int w1, int r2, int c2, int h2, int w2) =>
        r1 < r2 + h2 && r1 + h1 > r2 && c1 < c2 + w2 && c1 + w1 > c2;

    private static Color ColorForCluster(int id)
    {
        var hue = (id * 73) % 360;
        return ColorFromHsv(hue, 0.70f, 0.90f);
    }

    private static Color ColorFromHsv(float hue, float saturation, float value)
    {
        var h = hue / 60f; var c = value * saturation; var x = c * (1 - Math.Abs(h % 2 - 1)); var m = value - c;
        (float r, float g, float b) = h switch { < 1 => (c, x, 0f), < 2 => (x, c, 0f), < 3 => (0f, c, x), < 4 => (0f, x, c), < 5 => (x, 0f, c), _ => (c, 0f, x) };
        return Color.FromArgb((int)((r + m) * 255), (int)((g + m) * 255), (int)((b + m) * 255));
    }

    private static bool TryParseDriverId(string text, out AxisAddress address)
    {
        if (AxisAddress.TryParse(text, out address)) return true;
        if (int.TryParse(text.Trim(), out var n) && n is >= 1 and <= 64)
        {
            address = new AxisAddress((n - 1) / 16 + 1, (n - 1) % 16 + 1);
            return true;
        }
        address = default;
        return false;
    }

    private void SaveClusterConfiguration()
    {
        if (_loadingClusterConfig || IsDisposed)
        {
            return;
        }

        try
        {
            var config = new AutoClusterConfigFile
            {
                Version = 1,
                SelectedClusterId = _selectedClusterId,
                NextClusterId = Math.Max(
                    _nextClusterId,
                    _clusters.Count == 0 ? 1 : _clusters.Max(c => c.Id) + 1),
                Clusters = _clusters
                    .OrderBy(c => c.Id)
                    .Select(cluster => new AutoClusterConfigItem
                    {
                        Id = cluster.Id,
                        TopRow = cluster.TopRow,
                        LeftColumn = cluster.LeftColumn,
                        Width = cluster.Width,
                        Height = cluster.Height,
                        Effect = cluster.Effect,
                        WaveDirection = cluster.WaveDirection,
                        LayerOffsetRevolutions = cluster.LayerOffsetRevolutions,
                        FrequencyHz = cluster.FrequencyHz,
                        LidarRandomSeed = cluster.LidarRandomSeed,
                        Drivers = cluster.Drivers
                            .Where(pair => pair.Value is AxisAddress)
                            .Select(pair =>
                            {
                                var address = pair.Value!.Value;
                                return new AutoClusterDriverBinding
                                {
                                    Row = pair.Key.Row,
                                    Column = pair.Key.Column,
                                    Line = address.Line,
                                    SlaveId = address.SlaveId
                                };
                            })
                            .OrderBy(binding => binding.Row)
                            .ThenBy(binding => binding.Column)
                            .ToList()
                    })
                    .ToList()
            };

            var directory = Path.GetDirectoryName(_clusterConfigFilePath);
            if (!string.IsNullOrWhiteSpace(directory))
            {
                Directory.CreateDirectory(directory);
            }

            var json = JsonSerializer.Serialize(
                config,
                new JsonSerializerOptions
                {
                    WriteIndented = true
                });

            // Ghi file tạm rồi replace để hạn chế JSON hỏng khi app đóng đột ngột.
            var tempFile = _clusterConfigFilePath + ".tmp";
            File.WriteAllText(tempFile, json);
            File.Move(tempFile, _clusterConfigFilePath, overwrite: true);
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[AUTO CONFIG] Không lưu được cấu hình cụm: {ex.Message}");
        }
    }

    private void LoadClusterConfiguration()
    {
        if (!File.Exists(_clusterConfigFilePath))
        {
            return;
        }

        _loadingClusterConfig = true;

        try
        {
            var json = File.ReadAllText(_clusterConfigFilePath);
            var config = JsonSerializer.Deserialize<AutoClusterConfigFile>(
                json,
                new JsonSerializerOptions
                {
                    PropertyNameCaseInsensitive = true
                });

            if (config is null)
            {
                return;
            }

            var loadedClusters = new List<ClusterDraft>();
            var usedDrivers = new HashSet<AxisAddress>();

            foreach (var saved in config.Clusters.OrderBy(c => c.Id))
            {
                if (saved.Id <= 0 ||
                    saved.Width is < 1 or > 16 ||
                    saved.Height is < 1 or > 16 ||
                    saved.TopRow < 0 ||
                    saved.LeftColumn < 0 ||
                    saved.TopRow + saved.Height > 16 ||
                    saved.LeftColumn + saved.Width > 16)
                {
                    _state.WriteLog(
                        LogLevel.Warning,
                        $"[AUTO CONFIG] Bỏ qua Cụm {saved.Id}: kích thước/vị trí không hợp lệ.");
                    continue;
                }

                if (loadedClusters.Any(other =>
                    RectsOverlap(
                        saved.TopRow,
                        saved.LeftColumn,
                        saved.Height,
                        saved.Width,
                        other.TopRow,
                        other.LeftColumn,
                        other.Height,
                        other.Width)))
                {
                    _state.WriteLog(
                        LogLevel.Warning,
                        $"[AUTO CONFIG] Bỏ qua Cụm {saved.Id}: chồng lên cụm đã load.");
                    continue;
                }

                var draft = new ClusterDraft
                {
                    Id = saved.Id,
                    TopRow = saved.TopRow,
                    LeftColumn = saved.LeftColumn,
                    Width = saved.Width,
                    Height = saved.Height,
                    Effect = saved.Effect,
                    WaveDirection = saved.WaveDirection,
                    LayerOffsetRevolutions = Math.Clamp(saved.LayerOffsetRevolutions, 0.001, 1.0),
                    FrequencyHz = Math.Clamp(saved.FrequencyHz, 0.01, 5.0),
                    LidarRandomSeed = saved.LidarRandomSeed == 0
                        ? Random.Shared.Next(1, int.MaxValue)
                        : saved.LidarRandomSeed
                };

                foreach (var cell in draft.Cells())
                {
                    draft.Drivers[cell] = null;
                }

                foreach (var binding in saved.Drivers)
                {
                    var cell = (binding.Row, binding.Column);

                    if (!draft.Contains(cell.Row, cell.Column) ||
                        binding.Line is < 1 or > 4 ||
                        binding.SlaveId is < 1 or > 16)
                    {
                        continue;
                    }

                    var address = new AxisAddress(
                        binding.Line,
                        binding.SlaveId);

                    if (!usedDrivers.Add(address))
                    {
                        _state.WriteLog(
                            LogLevel.Warning,
                            $"[AUTO CONFIG] Driver {address.DisplayId} bị trùng; bỏ binding trùng.");
                        continue;
                    }

                    draft.Drivers[cell] = address;
                }

                loadedClusters.Add(draft);
            }

            _clusters.Clear();
            _clusters.AddRange(loadedClusters);

            _nextClusterId = Math.Max(
                Math.Max(1, config.NextClusterId),
                _clusters.Count == 0 ? 1 : _clusters.Max(c => c.Id) + 1);

            _selectedClusterId =
                _clusters.Any(c => c.Id == config.SelectedClusterId)
                    ? config.SelectedClusterId
                    : _clusters.FirstOrDefault()?.Id ?? 0;

            RebuildClusterCombo();
            LoadSelectedClusterSettings();
            RefreshGrid();
            RefreshLidarSimulationControls();
            RefreshAutoReadiness();

            _state.WriteLog(
                LogLevel.Ok,
                $"[AUTO CONFIG] Đã khôi phục {_clusters.Count} cụm từ file cấu hình.");
        }
        catch (Exception ex)
        {
            _state.WriteLog(
                LogLevel.Warning,
                $"[AUTO CONFIG] Không đọc được cấu hình cụm; dùng cấu hình trống. {ex.Message}");
        }
        finally
        {
            _loadingClusterConfig = false;
        }
    }

    private static AutoCluster ToAutoCluster(ClusterDraft c) => new(
        c.Id,
        c.TopRow,
        c.LeftColumn,
        c.Width,
        c.Height,
        c.Cells().Select(cell => new AutoGridCell(cell.Row, cell.Column, c.Drivers[cell])).ToArray(),
        c.Effect,
        c.WaveDirection,
        c.LayerOffsetRevolutions,
        c.FrequencyHz,
        c.LidarRandomSeed);

    private void RebuildProgramData()
    {
        // Dùng trực tiếp dữ liệu draft trong UI; không cần đồng bộ với các mode khác.
    }

    private AutoProgram? TryBuildProgram()
    {
        if (_clusters.Count == 0) return null;
        var cells = new List<AutoGridCell>();
        foreach (var cluster in _clusters)
            foreach (var cell in cluster.Cells())
            {
                cluster.Drivers.TryGetValue(cell, out var driver);
                cells.Add(new AutoGridCell(cell.Row, cell.Column, driver));
            }

        var clusters = _clusters.Select(c => new AutoCluster(
            c.Id,
            c.TopRow,
            c.LeftColumn,
            c.Width,
            c.Height,
            c.Cells().Select(cell => new AutoGridCell(cell.Row, cell.Column, c.Drivers[cell])).ToArray(),
            c.Effect,
            c.WaveDirection,
            c.LayerOffsetRevolutions,
            c.FrequencyHz,
            c.LidarRandomSeed)).ToArray();

        return new AutoProgram(16, 16, cells, clusters, GetConfiguredPpr(), (double)_frequency.Value, (double)_layerOffset.Value, (double)_rampUp.Value, (double)_rampDown.Value);
    }

    private int GetConfiguredPpr()
    {
        var selected = _state.Axes.FirstOrDefault(a => a.IsOnline);
        if (selected is null) return 10000;
        // AUTO thật sẽ lấy PPR riêng từng driver trong service; giá trị này chỉ dùng cho preview/model.
        return 10000;
    }

    private async Task SetSelectedClusterOriginAsync()
    {
        if (_autoRunning)
        {
            MessageBox.Show(this,
                "Hãy STOP AUTO trước khi lấy vị trí hiện tại làm gốc.",
                "AUTO ORIGIN",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning);
            return;
        }

        var cluster = SelectedCluster();
        if (cluster is null)
        {
            MessageBox.Show(this,
                "Hãy chọn một cụm trước.",
                "AUTO ORIGIN",
                MessageBoxButtons.OK,
                MessageBoxIcon.Information);
            return;
        }

        var missing = cluster.Cells()
            .Where(cell => !cluster.Drivers.TryGetValue(cell, out var driver) || driver is null)
            .ToArray();
        if (missing.Length > 0)
        {
            MessageBox.Show(this,
                $"Cụm {cluster.Id} còn {missing.Length} ô chưa gán Driver ID.",
                "AUTO ORIGIN",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning);
            return;
        }

        var drivers = cluster.Cells()
            .Select(cell => cluster.Drivers[cell]!.Value)
            .Distinct()
            .ToArray();
        var offline = drivers.Where(address => !_state.GetAxis(address).IsOnline).ToArray();
        if (offline.Length > 0)
        {
            MessageBox.Show(this,
                $"Không thể lấy gốc vì có driver offline: {string.Join(", ", offline.Select(a => a.DisplayId))}.",
                "AUTO ORIGIN",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning);
            return;
        }

        var confirm = MessageBox.Show(this,
            $"Đặt vị trí HIỆN TẠI của toàn bộ {drivers.Length} driver trong Cụm {cluster.Id} thành pha 0?\n\n" +
            "Hãy chắc chắn cơ cấu đang ở vị trí bạn muốn dùng làm gốc trước khi xác nhận.",
            "Xác nhận lấy gốc AUTO",
            MessageBoxButtons.YesNo,
            MessageBoxIcon.Warning,
            MessageBoxDefaultButton.Button2);
        if (confirm != DialogResult.Yes)
            return;

        try
        {
            _autoState.Text = "SETTING ORIGIN...";
            _autoState.ForeColor = UiTheme.Warning;
            await _service.SetCurrentPositionAsOriginAsync(drivers);
            _state.WriteLog(LogLevel.Ok,
                $"AUTO: Cụm {cluster.Id} đã lấy vị trí hiện tại làm pha 0 cho {drivers.Length} driver.");
        }
        catch (Exception ex)
        {
            _autoState.Text = "ORIGIN ERROR";
            _autoState.ForeColor = UiTheme.Error;
            _state.WriteLog(LogLevel.Error, $"AUTO ORIGIN lỗi: {ex.Message}");
            MessageBox.Show(this, ex.Message, "AUTO ORIGIN", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
        finally
        {
            RefreshGrid();
        }
    }

    private async Task StartAutoAsync()
    {
        // Không cho START khi STOP đang xử lý.
        if (_autoStopInProgress)
        {
            _state.WriteLog(
                LogLevel.Warning,
                "AUTO đang STOP; hãy chờ STOP hoàn tất.");
            return;
        }

        // Không cho chạy thêm một START khác khi START cũ vẫn còn sống.
        if (_autoStartTask is { IsCompleted: false })
        {
            _state.WriteLog(
                LogLevel.Warning,
                "AUTO đang trong quá trình STARTING...");
            return;
        }

        if (_autoRunning)
        {
            _state.WriteLog(
                LogLevel.Warning,
                "AUTO đang chạy; hãy STOP trước khi START lại.");
            return;
        }

        var program = TryBuildProgram();

        if (program is null)
        {
            _autoState.Text = "INVALID";
            _autoState.ForeColor = UiTheme.Error;

            _state.WriteLog(
                LogLevel.Error,
                "AUTO START: chưa tạo cụm.");

            return;
        }

        var readiness = GetAutoReadiness();

        if (!readiness.Ready)
        {
            _autoState.Text = "NOT READY";
            _autoState.ForeColor = UiTheme.Warning;

            _state.WriteLog(
                LogLevel.Error,
                $"AUTO START bị khóa: {readiness.Message}.");

            MessageBox.Show(
                this,
                $"Chưa thể chạy AUTO.\n\n" +
                $"{readiness.Message}\n\n" +
                "Mỗi cụm phải đủ Driver ID, 100% Online và đã HOME " +
                "hoặc lấy vị trí hiện tại làm gốc.",
                "AUTO chưa sẵn sàng",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning);

            return;
        }

        // ============================================================
        // TẠO TOKEN RIÊNG CHO LẦN START NÀY
        // ============================================================

        var startCts = new CancellationTokenSource();

        _autoStartCts = startCts;

        _autoRunning = true;
        _paused = false;

        _preview.ResetTime();
        _preview.Running = true;
        _preview.Paused = false;

        _autoState.Text = "STARTING...";
        _autoState.ForeColor = UiTheme.Accent;

        Task? startTask = null;

        try
        {
            // QUAN TRỌNG:
            // Truyền CancellationToken xuống service.
            startTask = _service.StartAutoAsync(
                program,
                startCts.Token);

            _autoStartTask = startTask;

            // Đợi toàn bộ:
            // PRE-PHASE
            // → WAIT POSITION
            // → CONFIG 16PR
            // → TRIGGER START
            await startTask;

            // STOP có thể vừa được nhấn đúng lúc service hoàn tất.
            // Không được đổi trạng thái trở lại RUNNING.
            if (startCts.IsCancellationRequested ||
                _autoStopInProgress)
            {
                return;
            }

            _autoState.Text = "RUNNING";
            _autoState.ForeColor = UiTheme.Online;
            RefreshLidarSimulationControls();
        }
        catch (OperationCanceledException)
            when (startCts.IsCancellationRequested)
        {
            // Đây là trường hợp người dùng bấm STOP trong lúc STARTING.
            // Không coi đây là ERROR.

            _state.WriteLog(
                LogLevel.Warning,
                "AUTO START đã bị hủy bởi STOP.");

            _autoRunning = false;
            _paused = false;

            _preview.Running = false;
            _preview.Paused = false;

            // Không ghi STOPPED ở đây.
            // StopAutoAsync() sẽ quyết định trạng thái cuối cùng.
        }
        catch (Exception ex)
        {
            _autoRunning = false;
            _paused = false;

            _preview.Running = false;
            _preview.Paused = false;

            // Nếu STOP đang diễn ra thì không bật popup ERROR
            // gây khó chịu cho người vận hành.
            if (!_autoStopInProgress)
            {
                _autoState.Text = "ERROR";
                _autoState.ForeColor = UiTheme.Error;

                _state.WriteLog(
                    LogLevel.Error,
                    $"AUTO START lỗi: {ex.Message}");

                MessageBox.Show(
                    this,
                    ex.Message,
                    "AUTO START",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
            }
        }
        finally
        {
            // Chỉ xóa nếu đây vẫn là lần START hiện tại.
            if (ReferenceEquals(_autoStartTask, startTask))
            {
                _autoStartTask = null;
            }

            if (ReferenceEquals(_autoStartCts, startCts))
            {
                _autoStartCts = null;
            }

            startCts.Dispose();
        }
    }

    private async Task TogglePauseAsync()
    {
        if (!_autoRunning)
        {
            _state.WriteLog(LogLevel.Warning, "AUTO chưa chạy.");
            return;
        }
        _paused = !_paused;
        _preview.Paused = _paused;
        _autoState.Text = _paused ? "PAUSED" : "RUNNING";
        await _service.PauseAutoAsync(_paused);
        RefreshLidarSimulationControls();
    }

    private async Task StopAutoAsync(bool quick)
    {
       
        if (_autoStopInProgress)
        {
            return;
        }

        _autoStopInProgress = true;

        try
        {
            _autoState.Text =
                quick ? "QUICK STOPPING..." : "STOPPING...";

            _autoState.ForeColor =
                quick ? UiTheme.Error : UiTheme.Warning;

            _paused = false;

            _preview.Paused = false;
            _preview.Running = false;

         
            var startCts = _autoStartCts;
            var startTask = _autoStartTask;

            if (startCts is not null &&
                !startCts.IsCancellationRequested)
            {
                startCts.Cancel();

                _state.WriteLog(
                    LogLevel.Warning,
                    "AUTO STOP: đang hủy quá trình START...");
            }

  

            if (startTask is not null &&
                !startTask.IsCompleted)
            {
                try
                {
                    await startTask;
                }
                catch (OperationCanceledException)
                {
                
                }
                catch (Exception ex)
                {
                   
                    _state.WriteLog(
                        LogLevel.Warning,
                        $"AUTO STOP: task START kết thúc với lỗi: {ex.Message}");
                }
            }


            _autoRunning = false;
            _paused = false;
            CancelLidarUiWindowTimers();
            _lidarActiveZones.Clear();

            await _service.StopAllAsync(quick);

            _autoState.Text =
                quick ? "QUICK STOP" : "STOPPED";

            _autoState.ForeColor =
                quick ? UiTheme.Error : UiTheme.Warning;

            _state.WriteLog(
                LogLevel.Ok,
                quick
                    ? "AUTO QUICK STOP hoàn tất."
                    : "AUTO STOP hoàn tất.");
        }
        catch (Exception ex)
        {
            _autoRunning = false;
            _paused = false;

            _preview.Running = false;
            _preview.Paused = false;

            _autoState.Text = "STOP ERROR";
            _autoState.ForeColor = UiTheme.Error;

            _state.WriteLog(
                LogLevel.Error,
                $"AUTO STOP lỗi: {ex.Message}");

            MessageBox.Show(
                this,
                ex.Message,
                "AUTO STOP",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);
        }
        finally
        {
            _autoStopInProgress = false;

            RefreshAutoReadiness();
            RefreshLidarSimulationControls();
        }
    }


    private void RefreshLidarSimulationControls()
    {
        var cluster = SelectedCluster();
        var isLidar = cluster is not null && cluster.Effect == AutoEffectType.Lidar;

        var previous = _lidarZoneCombo.SelectedItem is LidarZoneItem item ? item.Index : 0;
        _lidarZoneCombo.Items.Clear();
        if (isLidar && cluster is not null)
        {
            for (var i = 0; i < cluster.Width; i++)
                _lidarZoneCombo.Items.Add(new LidarZoneItem(i, $"Zone {i + 1} → Cột {i + 1}"));

            if (_lidarZoneCombo.Items.Count > 0)
                _lidarZoneCombo.SelectedIndex = Math.Clamp(previous, 0, _lidarZoneCombo.Items.Count - 1);
        }

        _lidarZoneCombo.Enabled = isLidar;
        var canCommand = isLidar && _autoRunning && !_paused && !_autoStopInProgress;
        var waveLocked = cluster is not null &&
                         _lidarActiveZones.TryGetValue(cluster.Id, out var activeZone) &&
                         activeZone is int;
        _lidarEnterButton.Enabled = canCommand && !waveLocked;
        // Trong 30 giây boost, giữ Zone hiện tại để tránh ghi chồng tốc độ.
        _lidarExitButton.Enabled = canCommand && !waveLocked;
    }

    private async Task SimulateLidarZoneEnterAsync()
    {
        var cluster = SelectedCluster();
        if (cluster is null || cluster.Effect != AutoEffectType.Lidar)
            return;
        if (!_autoRunning)
        {
            _state.WriteLog(LogLevel.Warning, "LIDAR TEST: hãy AUTO START trước.");
            return;
        }
        if (_lidarZoneCombo.SelectedItem is not LidarZoneItem zone)
            return;

        if (_lidarActiveZones.TryGetValue(cluster.Id, out var locked) && locked is int lockedZone)
        {
            _state.WriteLog(LogLevel.Info,
                $"LIDAR TEST: Cụm {cluster.Id} đang BOOST Zone {lockedZone + 1} 2X trong 30 giây; bỏ qua Zone mới.");
            return;
        }

        try
        {
            _autoState.Text = $"LIDAR Z{zone.Index + 1} · BOOST 2X...";
            _autoState.ForeColor = UiTheme.Accent;
            await _service.SetLidarZoneAsync(cluster.Id, zone.Index);
            _lidarActiveZones[cluster.Id] = zone.Index;
            _autoState.Text = $"LIDAR Z{zone.Index + 1} · BOOST 2X · 30s";
            _autoState.ForeColor = UiTheme.Online;
            RefreshLidarSimulationControls();
            RefreshGrid();
            _preview.Invalidate();
            StartLidarUiWindowTimer(cluster.Id, zone.Index);
        }
        catch (OperationCanceledException)
        {
            // STOP đã hủy transition.
        }
        catch (Exception ex)
        {
            _autoState.Text = "LIDAR ERROR";
            _autoState.ForeColor = UiTheme.Error;
            _state.WriteLog(LogLevel.Error, $"LIDAR TEST lỗi: {ex.Message}");
            MessageBox.Show(this, ex.Message, "LIDAR TEST", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    private async Task SimulateLidarZoneExitAsync()
    {
        var cluster = SelectedCluster();
        if (cluster is null || cluster.Effect != AutoEffectType.Lidar)
            return;
        if (!_autoRunning)
            return;

        if (_lidarActiveZones.TryGetValue(cluster.Id, out var activeZone) && activeZone is int zone)
        {
            _state.WriteLog(LogLevel.Info,
                $"LIDAR TEST: Zone {zone + 1} đang BOOST 2X trong 30 giây; EXIT thủ công bị bỏ qua.");
            return;
        }

        try
        {
            await _service.SetLidarZoneAsync(cluster.Id, null);
            _autoState.Text = "RUNNING · LIDAR RANDOM";
            _autoState.ForeColor = UiTheme.Online;
            RefreshGrid();
            _preview.Invalidate();
        }
        catch (OperationCanceledException)
        {
            // STOP đã hủy transition.
        }
        catch (Exception ex)
        {
            _autoState.Text = "LIDAR ERROR";
            _autoState.ForeColor = UiTheme.Error;
            _state.WriteLog(LogLevel.Error, $"LIDAR EXIT lỗi: {ex.Message}");
            MessageBox.Show(this, ex.Message, "LIDAR TEST", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    private void StartLidarUiWindowTimer(int clusterId, int zoneIndex)
    {
        if (_lidarUiWindowCts.Remove(clusterId, out var oldCts))
        {
            try { oldCts.Cancel(); } catch { }
            oldCts.Dispose();
        }

        var cts = new CancellationTokenSource();
        _lidarUiWindowCts[clusterId] = cts;
        _ = CompleteLidarUiWindowAsync(clusterId, zoneIndex, cts);
    }

    private async Task CompleteLidarUiWindowAsync(
        int clusterId,
        int zoneIndex,
        CancellationTokenSource cts)
    {
        try
        {
            await Task.Delay(TimeSpan.FromSeconds(30), cts.Token);
            if (cts.IsCancellationRequested || !_autoRunning)
                return;

            if (_lidarActiveZones.TryGetValue(clusterId, out var activeZone) && activeZone == zoneIndex)
            {
                _lidarActiveZones[clusterId] = null;
                _autoState.Text = "LIDAR BOOST DONE · RANDOM 1X";
                _autoState.ForeColor = UiTheme.Accent;
                RefreshLidarSimulationControls();
                RefreshGrid();
                _preview.Invalidate();
            }
        }
        catch (OperationCanceledException)
        {
            // STOP hoặc boost mới đã thay timer UI.
        }
        finally
        {
            if (_lidarUiWindowCts.TryGetValue(clusterId, out var current) && ReferenceEquals(current, cts))
            {
                _lidarUiWindowCts.Remove(clusterId);
            }
            cts.Dispose();
        }
    }

    private void CancelLidarUiWindowTimers()
    {
        foreach (var cts in _lidarUiWindowCts.Values.ToArray())
        {
            try { cts.Cancel(); } catch { }
            cts.Dispose();
        }
        _lidarUiWindowCts.Clear();
    }

    private void UpdateInspectValue()
    {
        if (AxisAddress.TryParse(_inspectAxis.SelectedItem?.ToString(), out var axis))
        {
            _preview.InspectAxis = axis;
            _inspectValue.Text = $"{_preview.InspectPositionRevolutions:0.000} vòng · {_preview.InspectPhaseDegrees:0.0}°";
        }
    }
    private AutoEffectType SelectedEffect() =>
        _effectCombo.SelectedItem is EffectItem item
            ? item.Value
            : AutoEffectType.WaveFromCenter;

    private AutoWaveDirection SelectedWaveDirection() =>
        _waveDirectionCombo.SelectedItem is DirectionItem item
            ? item.Value
            : AutoWaveDirection.LeftToRight;

    private void SelectEffect(AutoEffectType effect)
    {
        for (var i = 0; i < _effectCombo.Items.Count; i++)
        {
            if (_effectCombo.Items[i] is EffectItem item && item.Value == effect)
            {
                _effectCombo.SelectedIndex = i;
                return;
            }
        }

        _effectCombo.SelectedIndex = 0;
    }

    private void SelectWaveDirection(AutoWaveDirection direction)
    {
        for (var i = 0; i < _waveDirectionCombo.Items.Count; i++)
        {
            if (_waveDirectionCombo.Items[i] is DirectionItem item && item.Value == direction)
            {
                _waveDirectionCombo.SelectedIndex = i;
                return;
            }
        }

        _waveDirectionCombo.SelectedIndex = 0;
    }

    private static string EffectSummary(ClusterDraft cluster)
    {
        if (cluster.Effect == AutoEffectType.WaveFromCenter)
            return "Tâm";
        if (cluster.Effect == AutoEffectType.Lidar)
            return "LIDAR";

        var direction = cluster.WaveDirection switch
        {
            AutoWaveDirection.LeftToRight => "Trái→Phải",
            AutoWaveDirection.RightToLeft => "Phải→Trái",
            AutoWaveDirection.TopToBottom => "Trên→Dưới",
            AutoWaveDirection.BottomToTop => "Dưới→Trên",
            _ => "Đầu→Cuối"
        };
        return $"Đầu→Cuối {direction}";
    }

    private sealed record EffectItem(AutoEffectType Value, string Text)
    {
        public override string ToString() => Text;
    }

    private sealed record DirectionItem(AutoWaveDirection Value, string Text)
    {
        public override string ToString() => Text;
    }

    private sealed record LidarZoneItem(int Index, string Text)
    {
        public override string ToString() => Text;
    }

    private sealed record ClusterItem(int Id, string Text)
    {
        public override string ToString() => Text;
    }

    private static Label NewValueLabel(string text = "—")
    {
        var label = UiTheme.Label(text, new Font("Segoe UI Semibold", 10.5F, FontStyle.Bold));
        label.AutoSize = false;
        label.TextAlign = ContentAlignment.MiddleLeft;
        return label;
    }

    private static Control BuildHeader(string title, string subtitle)
    {
        var layout = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 2, ColumnCount = 1, BackColor = Color.Transparent };
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 55));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 45));
        var titleLabel = UiTheme.Label(title, UiTheme.FontTitle, UiTheme.Text);
        var subLabel = UiTheme.Label(subtitle, UiTheme.FontSmall, UiTheme.Muted);
        titleLabel.Dock = DockStyle.Fill; subLabel.Dock = DockStyle.Fill;
        titleLabel.AutoSize = false; subLabel.AutoSize = false;
        titleLabel.TextAlign = ContentAlignment.MiddleLeft; subLabel.TextAlign = ContentAlignment.MiddleLeft;
        layout.Controls.Add(titleLabel, 0, 0); layout.Controls.Add(subLabel, 0, 1);
        return layout;
    }

    private static Control LabeledControl(string text, Control control, string suffix = "")
    {
        var panel = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2, BackColor = Color.Transparent };
        panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 20));
        panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        panel.Controls.Add(UiTheme.Label(string.IsNullOrEmpty(suffix) ? text : $"{text} ({suffix})", UiTheme.FontSmall, UiTheme.Muted), 0, 0);
        panel.Controls.Add(control, 0, 1);
        return panel;
    }

    private static Control ValueCard(string title, Label value)
    {
        var panel = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 2, ColumnCount = 1, BackColor = UiTheme.SurfaceAlt, Padding = new Padding(8), Margin = new Padding(2) };
        panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 20));
        panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        panel.Controls.Add(UiTheme.Label(title, UiTheme.FontSmall, UiTheme.Muted), 0, 0);
        panel.Controls.Add(value, 0, 1);
        return panel;
    }

    private void BeginInvokeSafe(Action action)
    {
        if (IsDisposed || !IsHandleCreated) return;
        try { BeginInvoke(action); } catch { }
    }

    private void AutoPage_Load_1(object sender, EventArgs e)
    {

    }
    private int GetNextAvailableClusterId()
    {
        var id = 1;

        while (_clusters.Any(c => c.Id == id))
            id++;

        return id;
    }
}