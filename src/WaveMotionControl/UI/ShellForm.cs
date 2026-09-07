using System.ComponentModel;
using WaveMotionControl.Services;
using WaveMotionControl.State;
using WaveMotionControl.UI.Pages;

namespace WaveMotionControl.UI;

[DesignerCategory("Form")]
public partial class ShellForm : Form
{
    private readonly ApplicationState _state;
    private readonly IRs485Service _service;
    private readonly Panel _contentPanel;
    private readonly Dictionary<string, Button> _navButtons =
        new(StringComparer.OrdinalIgnoreCase);
    private readonly Dictionary<string, Control> _pages =
        new(StringComparer.OrdinalIgnoreCase);
    private readonly Label _systemStatus;
    private readonly Label _clockLabel;
    private readonly Label _footerStatus;
    private readonly System.Windows.Forms.Timer _clockTimer;

    public ShellForm() : this(CreateDefaultDependencies())
    {
    }

    private ShellForm(
        (ApplicationState State, IRs485Service Service) dependencies)
        : this(dependencies.State, dependencies.Service)
    {
    }

    public ShellForm(ApplicationState state, IRs485Service service)
    {
        _state = state ?? throw new ArgumentNullException(nameof(state));
        _service = service ?? throw new ArgumentNullException(nameof(service));

        InitializeComponent();

        SuspendLayout();

        // Các thiết lập này giúp bố cục ổn định hơn khi Windows Scale là
        // 100%, 125% hoặc 150%.
        AutoScaleMode = AutoScaleMode.Dpi;
        MinimumSize = new Size(1280, 720);
        StartPosition = FormStartPosition.CenterScreen;
        WindowState = FormWindowState.Maximized;
        BackColor = UiTheme.Background;
        SetStyle(ControlStyles.OptimizedDoubleBuffer, true);

        var root = new TableLayoutPanel
        {
            Name = "ShellRootLayout",
            Dock = DockStyle.Fill,
            RowCount = 3,
            ColumnCount = 1,
            BackColor = UiTheme.Background,
            Margin = Padding.Empty,
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        // Header cao hơn bản gốc để chữ trạng thái và đồng hồ không bị cắt.
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 88F));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 32F));

        var header = BuildHeader(out var systemStatus, out var clockLabel);
        _systemStatus = systemStatus;
        _clockLabel = clockLabel;

        _contentPanel = new Panel
        {
            Name = "ContentPanel",
            Dock = DockStyle.Fill,
            BackColor = UiTheme.Background,
            Padding = new Padding(10),
            Margin = Padding.Empty
        };

        var footer = BuildFooter(out var footerStatus);
        _footerStatus = footerStatus;

        root.Controls.Add(header, 0, 0);
        root.Controls.Add(_contentPanel, 0, 1);
        root.Controls.Add(footer, 0, 2);

        // Tránh giữ lại control cũ nếu Designer.cs từng thêm control vào Form.
        Controls.Clear();
        Controls.Add(root);

        ResumeLayout(performLayout: true);

        CreatePages();
        ShowPage("MAIN");

        _state.StateChanged += OnStateChanged;
        _state.LogAdded += OnLogAdded;

        _clockTimer = new System.Windows.Forms.Timer
        {
            Interval = 1000
        };
        _clockTimer.Tick += ClockTimer_Tick;
        _clockTimer.Start();

        UpdateClock();
        UpdateSystemStatus();

        FormClosed += ShellForm_FormClosed;
    }

    private static (ApplicationState State, IRs485Service Service)
        CreateDefaultDependencies()
    {
        var state = new ApplicationState();
        return (state, new DemoRs485Service(state));
    }

    private void ClockTimer_Tick(object? sender, EventArgs e)
    {
        UpdateClock();
    }

    private void ShellForm_FormClosed(object? sender, FormClosedEventArgs e)
    {
        _clockTimer.Stop();
        _clockTimer.Tick -= ClockTimer_Tick;
        _clockTimer.Dispose();

        _state.StateChanged -= OnStateChanged;
        _state.LogAdded -= OnLogAdded;
    }

    private void OnLogAdded(Models.LogEntry entry)
    {
        if (IsDisposed || Disposing || !IsHandleCreated)
        {
            return;
        }

        if (InvokeRequired)
        {
            try
            {
                BeginInvoke(new Action(() => SetFooterStatus(entry.Message)));
            }
            catch (InvalidOperationException)
            {
                // Form đang đóng hoặc handle đã được giải phóng.
            }

            return;
        }

        SetFooterStatus(entry.Message);
    }

    private void SetFooterStatus(string message)
    {
        if (!_footerStatus.IsDisposed)
        {
            _footerStatus.Text = message;
        }
    }

    private Panel BuildHeader(
        out Label systemStatus,
        out Label clockLabel)
    {
        var header = new Panel
        {
            Name = "HeaderPanel",
            Dock = DockStyle.Fill,
            BackColor = UiTheme.Surface,
            Padding = new Padding(16, 9, 16, 9),
            Margin = Padding.Empty
        };

        // Bốn vùng độc lập:
        // 1. Logo/tên, 2. menu, 3. trạng thái, 4. đồng hồ.
        // Cách này ngăn trạng thái bị đè bởi đồng hồ như bố cục ba cột cũ.
        var layout = new TableLayoutPanel
        {
            Name = "HeaderLayout",
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = 1,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 310F));
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 510F));
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 155F));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var brand = BuildBrandPanel();
        var navigation = BuildNavigationPanel();

        systemStatus = new Label
        {
            Name = "SystemStatusLabel",
            Text = "SYSTEM OFFLINE",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleCenter,
            BackColor = UiTheme.SurfaceAlt,
            ForeColor = UiTheme.Muted,
            Font = UiTheme.FontSection,
            Margin = new Padding(10, 7, 10, 7),
            Padding = new Padding(6, 2, 6, 2),
            BorderStyle = BorderStyle.FixedSingle,
            AutoEllipsis = true,
            AutoSize = false
        };

        clockLabel = new Label
        {
            Name = "ClockLabel",
            Text = "--:--:--\r\n--/--/----",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleRight,
            ForeColor = UiTheme.Text,
            Font = new Font(
                "Consolas",
                10.5F,
                FontStyle.Regular,
                GraphicsUnit.Point),
            BackColor = Color.Transparent,
            Margin = new Padding(4, 5, 0, 5),
            AutoSize = false
        };

        layout.Controls.Add(brand, 0, 0);
        layout.Controls.Add(navigation, 1, 0);
        layout.Controls.Add(systemStatus, 2, 0);
        layout.Controls.Add(clockLabel, 3, 0);

        header.Controls.Add(layout);
        return header;
    }

    private static Control BuildBrandPanel()
    {
        var brand = new TableLayoutPanel
        {
            Name = "BrandLayout",
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        brand.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 64F));
        brand.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        brand.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var logo = new Label
        {
            Name = "LogoLabel",
            Text = "64",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleCenter,
            BackColor = UiTheme.AccentDark,
            ForeColor = Color.White,
            Font = new Font(
                "Segoe UI Semibold",
                16F,
                FontStyle.Bold,
                GraphicsUnit.Point),
            Margin = new Padding(0, 1, 10, 1),
            AutoSize = false
        };

        // Không dùng Location tuyệt đối để tránh title/subtitle đè nhau khi DPI đổi.
        var titleStack = new TableLayoutPanel
        {
            Name = "BrandTextLayout",
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };
        titleStack.RowStyles.Add(new RowStyle(SizeType.Percent, 58F));
        titleStack.RowStyles.Add(new RowStyle(SizeType.Percent, 42F));

        var title = UiTheme.Label(
            "WAVE MOTION CONTROL",
            new Font(
                "Segoe UI Semibold",
                14.5F,
                FontStyle.Bold,
                GraphicsUnit.Point));
        title.Dock = DockStyle.Fill;
        title.TextAlign = ContentAlignment.BottomLeft;
        title.AutoSize = false;
        title.AutoEllipsis = true;
        title.Margin = Padding.Empty;

        var subtitle = UiTheme.Label(
            "4 line RS485 · 64 driver EM2RS",
            UiTheme.FontSmall,
            UiTheme.Muted);
        subtitle.Dock = DockStyle.Fill;
        subtitle.TextAlign = ContentAlignment.TopLeft;
        subtitle.AutoSize = false;
        subtitle.AutoEllipsis = true;
        subtitle.Margin = Padding.Empty;

        titleStack.Controls.Add(title, 0, 0);
        titleStack.Controls.Add(subtitle, 0, 1);

        brand.Controls.Add(logo, 0, 0);
        brand.Controls.Add(titleStack, 1, 0);

        return brand;
    }

    private Control BuildNavigationPanel()
    {
        var navigation = new TableLayoutPanel
        {
            Name = "NavigationLayout",
            Dock = DockStyle.Fill,
            ColumnCount = 5,
            RowCount = 1,
            BackColor = Color.Transparent,
            Margin = new Padding(6, 0, 6, 0),
            Padding = new Padding(0, 7, 0, 7),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };

        for (var column = 0; column < 5; column++)
        {
            navigation.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 20F));
        }
        navigation.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var pageNames = new[] { "MAIN", "MANUAL", "AUTO", "STATUS", "SETTING" };

        for (var index = 0; index < pageNames.Length; index++)
        {
            var pageName = pageNames[index];
            var button = UiTheme.Button(pageName);

            button.Name = $"Navigation{pageName}Button";
            button.Dock = DockStyle.Fill;
            button.Margin = new Padding(2, 0, 2, 0);
            button.AutoEllipsis = true;
            button.Font = new Font(
                "Segoe UI Semibold",
                9.5F,
                FontStyle.Bold,
                GraphicsUnit.Point);
            button.Click += (_, _) => ShowPage(pageName);

            _navButtons[pageName] = button;
            navigation.Controls.Add(button, index, 0);
        }

        return navigation;
    }

    private Panel BuildFooter(out Label footerStatus)
    {
        var footer = new Panel
        {
            Name = "FooterPanel",
            Dock = DockStyle.Fill,
            BackColor = UiTheme.Surface,
            Padding = new Padding(14, 0, 14, 0),
            Margin = Padding.Empty
        };

        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            BackColor = Color.Transparent,
            Margin = Padding.Empty,
            Padding = Padding.Empty
        };
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 65F));
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 35F));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        var left = new Label
        {
            Text = "Project: WAVE 4×16 · Modbus RTU · Thiết kế HMI 1920×1080",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = UiTheme.Muted,
            Font = UiTheme.FontSmall,
            AutoEllipsis = true,
            AutoSize = false,
            Margin = Padding.Empty
        };

        footerStatus = new Label
        {
            Name = "FooterStatusLabel",
            Text = "Ready",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleRight,
            ForeColor = UiTheme.Muted,
            Font = UiTheme.FontSmall,
            AutoEllipsis = true,
            AutoSize = false,
            Margin = Padding.Empty
        };

        layout.Controls.Add(left, 0, 0);
        layout.Controls.Add(footerStatus, 1, 0);
        footer.Controls.Add(layout);

        return footer;
    }

    private void CreatePages()
    {
        _pages["MAIN"] = new MainPage(_state, _service)
        {
            Dock = DockStyle.Fill,
            Margin = Padding.Empty
        };
        _pages["MANUAL"] = new ManualPage(_state, _service)
        {
            Dock = DockStyle.Fill,
            Margin = Padding.Empty
        };
        _pages["AUTO"] = new AutoPage(_state, _service)
        {
            Dock = DockStyle.Fill,
            Margin = Padding.Empty
        };
        _pages["STATUS"] = new StatusPage(_state)
        {
            Dock = DockStyle.Fill,
            Margin = Padding.Empty
        };
        _pages["SETTING"] = new SettingPageFixed(_state, _service)
        {
            Dock = DockStyle.Fill,
            Margin = Padding.Empty
        };

        _contentPanel.SuspendLayout();

        foreach (var page in _pages.Values)
        {
            page.Visible = false;
            _contentPanel.Controls.Add(page);
        }

        _contentPanel.ResumeLayout(performLayout: true);
    }

    private void ShowPage(string pageName)
    {
        if (!_pages.ContainsKey(pageName))
        {
            return;
        }

        _contentPanel.SuspendLayout();

        foreach (var pair in _pages)
        {
            var isActive = pair.Key.Equals(
                pageName,
                StringComparison.OrdinalIgnoreCase);

            pair.Value.Visible = isActive;

            if (isActive)
            {
                pair.Value.BringToFront();
            }
        }

        _contentPanel.ResumeLayout(performLayout: true);

        foreach (var pair in _navButtons)
        {
            var active = pair.Key.Equals(
                pageName,
                StringComparison.OrdinalIgnoreCase);

            pair.Value.BackColor =
                active ? UiTheme.AccentDark : UiTheme.SurfaceAlt;
            pair.Value.ForeColor =
                active ? Color.White : UiTheme.Text;
            pair.Value.FlatAppearance.BorderColor =
                active ? UiTheme.Accent : UiTheme.Border;
        }

        _footerStatus.Text = $"{pageName} screen";
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
                BeginInvoke(new Action(UpdateSystemStatus));
            }
            catch (InvalidOperationException)
            {
                // Form đang đóng hoặc handle đã được giải phóng.
            }

            return;
        }

        UpdateSystemStatus();
    }

    private void UpdateSystemStatus()
    {
        var connected = _state.Lines.Count(line => line.IsConnected);

        _systemStatus.Text = connected switch
        {
            0 => "SYSTEM OFFLINE",
            4 => "SYSTEM ONLINE",
            _ => $"SYSTEM PARTIAL {connected}/4"
        };

        _systemStatus.ForeColor = connected switch
        {
            4 => UiTheme.Online,
            > 0 => UiTheme.Warning,
            _ => UiTheme.Muted
        };
    }

    private void UpdateClock()
    {
        var now = DateTime.Now;
        _clockLabel.Text = $"{now:HH:mm:ss}\r\n{now:dd/MM/yyyy}";
    }

    private void ShellForm_Load(object? sender, EventArgs e)
    {
    }
}
