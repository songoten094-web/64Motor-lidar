using System.ComponentModel;
using WaveMotionControl.Models;
using WaveMotionControl.Services;
using WaveMotionControl.State;

namespace WaveMotionControl.UI.Pages;

/// <summary>
/// Trang Setting thay thế. Bố cục ba cột; riêng vùng giữa có AutoScroll
/// và các card xếp theo hàng, vì vậy không dùng BringToFront và không đè nhau.
/// </summary>
[DesignerCategory("UserControl")]
public sealed class SettingPageFixed : UserControl
{
    private const int CanvasWidth = 1450;
    private const int CanvasHeight = 820;

    private readonly ApplicationState _state;
    private readonly IRs485Service _service;
    private readonly IModeDriverSettingsService? _advanced;

    private readonly ComboBox _scope;
    private readonly ComboBox _line;
    private readonly ComboBox _axis;

    private readonly NumericUpDown _homeCurrent;
    private readonly NumericUpDown _manualCurrent;
    private readonly NumericUpDown _autoCurrent;
    private readonly NumericUpDown _ppr;
    private readonly NumericUpDown _homeFast;
    private readonly NumericUpDown _homeSlow;
    private readonly NumericUpDown _homeAcc;
    private readonly NumericUpDown _homeDec;

    private readonly ComboBox _di;
    private readonly ComboBox _polarity;
    private readonly NumericUpDown _standby;
    private readonly NumericUpDown _autoSpeed;
    private readonly NumericUpDown _autoAcc;

    private readonly Button _save;
    private readonly Button _read;
    private readonly Button _reset;
    private readonly RichTextBox _log;
    private readonly Label _status;
    private readonly Label _scopeValue;
    private readonly Label _driverValue;
    private readonly Label _eepromValue;
    private readonly Control _root;

    private bool _updatingSelection;
    private bool _operationRunning;

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

    public SettingPageFixed() : this(new DesignerDependencies())
    {
    }

    private SettingPageFixed(DesignerDependencies dependencies)
        : this(dependencies.State, dependencies.Service)
    {
    }

    public SettingPageFixed(ApplicationState state, IRs485Service service)
    {
        SuspendLayout();
        _state = state ?? throw new ArgumentNullException(nameof(state));
        _service = service ?? throw new ArgumentNullException(nameof(service));
        _advanced = service as IModeDriverSettingsService;

        AutoScaleMode = AutoScaleMode.Dpi;
        AutoScroll = true;
        AutoScrollMinSize = new Size(CanvasWidth, CanvasHeight);
        BackColor = UiTheme.Background;
        ForeColor = UiTheme.Text;
        Padding = new Padding(10);

        _scope = Combo(
            "TẤT CẢ DRIVER ONLINE",
            "MỘT LINE RS485",
            "MỘT DRIVER CỤ THỂ");
        _scope.SelectedIndex = 2;

        _line = UiTheme.ComboBox();
        for (var line = 1; line <= 4; line++)
        {
            _line.Items.Add($"Line {line} (Driver {line}.1–{line}.16)");
        }
        _line.SelectedIndex = 0;

        _axis = UiTheme.ComboBox();
        foreach (var address in AxisAddress.All())
        {
            _axis.Items.Add(address.DisplayId);
        }
        _axis.SelectedItem = _state.SelectedAxis.DisplayId;
        if (_axis.SelectedIndex < 0) _axis.SelectedIndex = 0;

        _homeCurrent = Current(3.0M);
        _manualCurrent = Current(3.0M);
        _autoCurrent = Current(3.0M);
        _ppr = Number(10_000, 200, 51_200, 100);

        _homeFast = Number(120, 1, 5000, 1);
        _homeSlow = Number(12, 1, 5000, 1);
        _homeAcc = Number(500, 1, 10_000, 10);
        _homeDec = Number(500, 1, 10_000, 10);

        _di = UiTheme.ComboBox();
        for (var pin = 1; pin <= 7; pin++) _di.Items.Add($"DI{pin}");
        _di.SelectedIndex = 4;

        _polarity = Combo(
            "N.O. (Thường mở / High Active)",
            "N.C. (Thường đóng / Low Active)");
        _polarity.SelectedIndex = 0;

        _standby = Number(35, 0, 100, 1);
        _autoSpeed = Number(600, 1, 5000, 10);
        _autoAcc = Number(50, 0.1M, 5000, 1, 1);

        _save = UiTheme.Button("LƯU TOÀN BỘ CẤU HÌNH", primary: true);
        _read = UiTheme.Button("ĐỌC CẤU HÌNH DRIVER");
        _reset = UiTheme.Button("KHÔI PHỤC THAM SỐ", danger: true);

        _save.Click += async (_, _) => await SaveAsync();
        _read.Click += async (_, _) => await ReadAsync();
        _reset.Click += async (_, _) => await ResetAsync();

        _log = new RichTextBox
        {
            Dock = DockStyle.Fill,
            ReadOnly = true,
            BackColor = UiTheme.Background,
            ForeColor = UiTheme.Text,
            BorderStyle = BorderStyle.FixedSingle,
            Font = new Font("Consolas", 9F),
            WordWrap = true
        };

        _scopeValue = SummaryValue("MỘT DRIVER CỤ THỂ");
        _driverValue = SummaryValue(_state.SelectedAxis.DisplayId);
        _eepromValue = SummaryValue("CHƯA LƯU");
        _save.Enabled = _advanced is not null;
        _read.Enabled = _advanced is not null;

        _status = new Label
        {
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = _advanced is null ? UiTheme.Error : UiTheme.Muted,
            Font = UiTheme.FontSmall,
            Text = _advanced is null
                ? "Service chưa hỗ trợ IModeDriverSettingsService."
                : "Sẵn sàng. Dòng bị giới hạn cứng tối đa 4,0 A."
        };

        _root = BuildRoot();
        Controls.Add(_root);

        _scope.SelectedIndexChanged += (_, _) => ApplyScopeFromUi();
        _line.SelectedIndexChanged += (_, _) => ApplyScopeFromUi();
        _axis.SelectedIndexChanged += (_, _) => ApplyScopeFromUi();
        Resize += OnPageResize;

        _state.LogAdded += OnLogAdded;
        _state.StateChanged += OnStateChanged;
        Disposed += (_, _) =>
        {
            _state.LogAdded -= OnLogAdded;
            _state.StateChanged -= OnStateChanged;
        };

        ApplyScopeFromUi();
        ResizeRoot();
        ResumeLayout(true);
    }

    private Control BuildRoot()
    {
        var root = new TableLayoutPanel
        {
            Location = new Point(Padding.Left, Padding.Top),
            MinimumSize = new Size(CanvasWidth - Padding.Horizontal,
                                   CanvasHeight - Padding.Vertical),
            ColumnCount = 3,
            RowCount = 1,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            BackColor = UiTheme.Background,
            Margin = new Padding(0),
            Padding = new Padding(0)
        };
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 285));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 340));

        root.Controls.Add(BuildScopeCard(), 0, 0);
        root.Controls.Add(BuildCenter(), 1, 0);
        root.Controls.Add(BuildManagementCard(), 2, 0);
        return root;
    }

    private Control BuildScopeCard()
    {
        var card = Card(new Padding(0, 0, 8, 0));
        var body = OneColumn(
            44, 34, 40, 34, 40, 34, 40, 135);

        body.Controls.Add(Title("PHẠM VI CÀI ĐẶT"), 0, 0);
        body.Controls.Add(SmallLabel("Áp dụng cấu hình cho"), 0, 1);
        body.Controls.Add(_scope, 0, 2);
        body.Controls.Add(SmallLabel("Chọn Line RS485"), 0, 3);
        body.Controls.Add(_line, 0, 4);
        body.Controls.Add(SmallLabel("Chọn ID Driver cụ thể"), 0, 5);
        body.Controls.Add(_axis, 0, 6);

        body.Controls.Add(new Label
        {
            Dock = DockStyle.Fill,
            Text =
                "LƯU Ý LƯU EEPROM\n\n" +
                "Motor phải dừng trước khi lưu. Khi đổi mapping DI, " +
                "tắt/bật lại nguồn driver một lần.",
            ForeColor = UiTheme.Accent,
            BackColor = UiTheme.SurfaceAlt,
            Padding = new Padding(12),
            Font = UiTheme.FontSmall
        }, 0, 7);

        card.Controls.Add(body);
        return card;
    }

    private Control BuildCenter()
    {
        var scroll = new Panel
        {
            Dock = DockStyle.Fill,
            AutoScroll = true,
            BackColor = UiTheme.Background,
            Padding = new Padding(4, 0, 8, 0),
            Margin = new Padding(0)
        };

        var stack = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 1,
            RowCount = 3,
            BackColor = UiTheme.Background,
            Margin = new Padding(0),
            Padding = new Padding(0)
        };
        stack.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        stack.RowStyles.Add(new RowStyle(SizeType.Absolute, 385));
        stack.RowStyles.Add(new RowStyle(SizeType.Absolute, 265));
        stack.RowStyles.Add(new RowStyle(SizeType.Absolute, 90));

        stack.Controls.Add(BuildModeHomeCard(), 0, 0);
        stack.Controls.Add(BuildSensorAutoCard(), 0, 1);
        stack.Controls.Add(BuildStatusCard(), 0, 2);
        scroll.Controls.Add(stack);

        void ResizeStack()
        {
            var width = scroll.ClientSize.Width -
                        scroll.Padding.Horizontal -
                        SystemInformation.VerticalScrollBarWidth;
            stack.Width = Math.Max(760, width);
        }

        scroll.Resize += (_, _) => ResizeStack();
        ResizeStack();
        return scroll;
    }

    private Control BuildModeHomeCard()
    {
        var card = Card(new Padding(0, 0, 0, 8));
        var grid = FourColumnGrid(7, 48, 45);

        AddFull(grid, Title("DÒNG THEO CHẾ ĐỘ, PULSE/VÒNG VÀ HOME"), 0);
        AddPair(grid, "Dòng HOME (A)", _homeCurrent, 0, 1);
        AddPair(grid, "Dòng MANUAL (A)", _manualCurrent, 2, 1);
        AddPair(grid, "Dòng AUTO (A)", _autoCurrent, 0, 2);
        AddPair(grid, "Pulse/vòng dùng chung", _ppr, 2, 2);
        AddPair(grid, "HOME nhanh (rpm)", _homeFast, 0, 3);
        AddPair(grid, "HOME chậm (rpm)", _homeSlow, 2, 3);
        AddPair(grid, "HOME Acc (ms/1000rpm)", _homeAcc, 0, 4);
        AddPair(grid, "HOME Dec (ms/1000rpm)", _homeDec, 2, 4);

        AddFull(grid, Note(
            "Dòng HOME được lưu EEPROM. MANUAL/AUTO được ghi vào 0x0191 " +
            "ngay trước khi chạy. Backend luôn chặn giá trị lớn hơn 4,0 A.",
            UiTheme.Muted), 5);

        AddFull(grid, Note(
            "Acc/Dec HOME dùng đơn vị ms/1000rpm: số lớn hơn = tăng/giảm tốc êm hơn.",
            UiTheme.Warning), 6);

        card.Controls.Add(grid);
        return card;
    }

    private Control BuildSensorAutoCard()
    {
        var card = Card(new Padding(0, 0, 0, 8));
        var grid = FourColumnGrid(5, 48, 45);

        AddFull(grid, Title("SENSOR HOME, DÒNG GIỮ VÀ THÔNG SỐ AUTO"), 0);
        AddPair(grid, "Chân cảm biến HOME", _di, 0, 1);
        AddPair(grid, "Kiểu kích hoạt", _polarity, 2, 1);
        AddPair(grid, "Dòng giữ Standby (%)", _standby, 0, 2);
        AddPair(grid, "Tốc độ AUTO (rpm)", _autoSpeed, 2, 2);
        AddPair(grid, "Gia tốc AUTO (vòng/s²)", _autoAcc, 0, 3);

        AddFull(grid, Note(
            "Khi lưu, service tạm dừng polling của line, gom các thanh ghi " +
            "liên tục bằng FC10 rồi đọc lại để xác nhận.",
            UiTheme.Muted), 4);

        card.Controls.Add(grid);
        return card;
    }

    private Control BuildStatusCard()
    {
        var card = Card(new Padding(0));
        var body = new Panel
        {
            Dock = DockStyle.Fill,
            Padding = new Padding(14),
            BackColor = Color.Transparent
        };
        body.Controls.Add(_status);
        card.Controls.Add(body);
        return card;
    }

    private Control BuildManagementCard()
    {
        var card = Card(new Padding(8, 0, 0, 0));
        var body = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 7,
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize,
            BackColor = Color.Transparent,
            Padding = new Padding(6),
            Margin = new Padding(0)
        };
        body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        body.RowStyles.Add(new RowStyle(SizeType.Absolute, 55F));
        body.RowStyles.Add(new RowStyle(SizeType.Absolute, 50F));
        body.RowStyles.Add(new RowStyle(SizeType.Absolute, 50F));
        body.RowStyles.Add(new RowStyle(SizeType.Absolute, 50F));
        body.RowStyles.Add(new RowStyle(SizeType.Absolute, 105F));
        body.RowStyles.Add(new RowStyle(SizeType.Absolute, 28F));
        body.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

        body.Controls.Add(Title("QUẢN LÝ EEPROM & LOG"), 0, 0);
        body.Controls.Add(_save, 0, 1);
        body.Controls.Add(_read, 0, 2);
        body.Controls.Add(_reset, 0, 3);
        body.Controls.Add(BuildSummary(), 0, 4);
        body.Controls.Add(SmallLabel("NHẬT KÝ EEPROM / MODBUS"), 0, 5);
        body.Controls.Add(_log, 0, 6);

        card.Controls.Add(body);
        return card;
    }

    private Control BuildSummary()
    {
        var grid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 2,
            BackColor = Color.Transparent,
            Margin = new Padding(0),
            Padding = new Padding(0)
        };
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
        grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50));

        grid.Controls.Add(SummaryCell("Phạm vi", _scopeValue), 0, 0);
        grid.Controls.Add(SummaryCell("Driver đọc", _driverValue), 1, 0);
        grid.Controls.Add(SummaryCell("Giới hạn dòng", SummaryValue("6,0 A")), 0, 1);
        grid.Controls.Add(SummaryCell("EEPROM", _eepromValue), 1, 1);
        return grid;
    }

    private async Task SaveAsync()
    {
        if (_advanced is null)
        {
            SetStatus(
                "Service chưa có SaveCompleteDriverSettingsAsync. " +
                "Hãy chép đủ ba file trong thư mục Services.",
                true);
            return;
        }

        var targets = ResolveTargets();
        if (targets.Length == 0)
        {
            SetStatus("Không có driver Online trong phạm vi đã chọn.", true);
            return;
        }

        await BusyAsync(async () =>
        {
            await _advanced.SaveCompleteDriverSettingsAsync(
                targets,
                _di.SelectedIndex + 1,
                _polarity.SelectedIndex == 1,
                (int)_standby.Value,
                (double)_autoSpeed.Value / 60.0,
                (double)_autoAcc.Value,
                BuildModeSettings());

            _eepromValue.Text = "ĐÃ GỬI / KIỂM TRA";
            SetStatus(
                $"Đã hoàn tất yêu cầu lưu {targets.Length} driver. " +
                "Xem log để biết kết quả từng driver.",
                false);
        });
    }

    private async Task ReadAsync()
    {
        if (_advanced is null)
        {
            SetStatus("Service chưa hỗ trợ cấu hình nâng cao.", true);
            return;
        }

        var address = SelectedAddress();
        if (!_state.GetAxis(address).IsOnline)
        {
            SetStatus($"Driver {address.DisplayId} đang Offline.", true);
            return;
        }

        await BusyAsync(async () =>
        {
            var basic = await _service.ReadDriverConfigAsync(address);
            var mode = await _advanced.ReadModeDriverSettingsAsync(address);

            _di.SelectedIndex = Math.Clamp(basic.diPinIndex - 1, 0, 6);
            _polarity.SelectedIndex = basic.activeLowNC ? 1 : 0;
            _standby.Value = Clamp(
                basic.standbyPercent,
                _standby.Minimum,
                _standby.Maximum);
            _autoSpeed.Value = Clamp(
                (decimal)(basic.autoSpeedRps * 60.0),
                _autoSpeed.Minimum,
                _autoSpeed.Maximum);
            _autoAcc.Value = Clamp(
                (decimal)basic.autoAccRps2,
                _autoAcc.Minimum,
                _autoAcc.Maximum);

            LoadModeSettings(mode);
            _driverValue.Text = address.DisplayId;
            _eepromValue.Text = "ĐÃ ĐỌC DRIVER";
            SetStatus($"Đã đọc cấu hình thực tế từ {address.DisplayId}.", false);
        });
    }

    private async Task ResetAsync()
    {
        var targets = ResolveTargets();
        if (targets.Length == 0)
        {
            SetStatus("Không có driver Online trong phạm vi đã chọn.", true);
            return;
        }

        if (MessageBox.Show(
                $"Khôi phục tham số cho {targets.Length} driver?\n" +
                "Cần tắt/bật nguồn driver sau thao tác.",
                "Xác nhận khôi phục",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning) != DialogResult.Yes)
        {
            return;
        }

        await BusyAsync(async () =>
        {
            await _service.ClearDriverConfigAsync(targets);
            _eepromValue.Text = "ĐÃ RESET";
            SetStatus("Đã gửi lệnh khôi phục. Tắt/bật lại nguồn driver.", false);
        });
    }

    private DriverModeSettings BuildModeSettings() => new(
        (double)_homeCurrent.Value,
        (double)_manualCurrent.Value,
        (double)_autoCurrent.Value,
        (int)_ppr.Value)
    {
        HomeFastSpeedRpm = (int)_homeFast.Value,
        HomeSlowSpeedRpm = (int)_homeSlow.Value,
        HomeAccelerationMsPer1000Rpm = (int)_homeAcc.Value,
        HomeDecelerationMsPer1000Rpm = (int)_homeDec.Value
    };

    private void LoadModeSettings(DriverModeSettings settings)
    {
        _homeCurrent.Value = Clamp(
            (decimal)settings.HomeCurrentAmps,
            _homeCurrent.Minimum,
            _homeCurrent.Maximum);
        _manualCurrent.Value = Clamp(
            (decimal)settings.ManualCurrentAmps,
            _manualCurrent.Minimum,
            _manualCurrent.Maximum);
        _autoCurrent.Value = Clamp(
            (decimal)settings.AutoCurrentAmps,
            _autoCurrent.Minimum,
            _autoCurrent.Maximum);
        _ppr.Value = Clamp(settings.PulsesPerRevolution, _ppr.Minimum, _ppr.Maximum);
        _homeFast.Value = Clamp(settings.HomeFastSpeedRpm, _homeFast.Minimum, _homeFast.Maximum);
        _homeSlow.Value = Clamp(settings.HomeSlowSpeedRpm, _homeSlow.Minimum, _homeSlow.Maximum);
        _homeAcc.Value = Clamp(settings.HomeAccelerationMsPer1000Rpm, _homeAcc.Minimum, _homeAcc.Maximum);
        _homeDec.Value = Clamp(settings.HomeDecelerationMsPer1000Rpm, _homeDec.Minimum, _homeDec.Maximum);
    }

    private AxisAddress[] ResolveTargets() =>
        _scope.SelectedIndex switch
        {
            0 => _state.Axes
                .Where(axis => axis.IsOnline)
                .Select(axis => axis.Address)
                .ToArray(),
            1 => _state.GetAxesForLine(_line.SelectedIndex + 1)
                .Where(axis => axis.IsOnline)
                .Select(axis => axis.Address)
                .ToArray(),
            _ => _state.GetAxis(SelectedAddress()).IsOnline
                ? new[] { SelectedAddress() }
                : Array.Empty<AxisAddress>()
        };

    private AxisAddress SelectedAddress()
    {
        var text = _axis.SelectedItem?.ToString() ?? "1.1";
        return AxisAddress.TryParse(text, out var address)
            ? address
            : new AxisAddress(1, 1);
    }

    private void ApplyScopeFromUi()
    {
        if (_updatingSelection || IsDisposed || Disposing)
        {
            return;
        }

        _line.Enabled = _scope.SelectedIndex == 1;
        _axis.Enabled = _scope.SelectedIndex == 2;

        var address = SelectedAddress();
        _scopeValue.Text = _scope.SelectedItem?.ToString() ?? "—";
        _driverValue.Text = address.DisplayId;

        // Không ghi lại SelectedAxis nếu giá trị không đổi. Một số bản ApplicationState
        // phát StateChanged trong setter; ghi lặp ở đây sẽ tạo vòng gọi UI liên tục.
        if (!_state.SelectedAxis.Equals(address))
        {
            _state.SelectedAxis = address;
        }
    }

    private void RefreshFromState()
    {
        if (IsDisposed || Disposing)
        {
            return;
        }

        _updatingSelection = true;
        try
        {
            var selected = _state.SelectedAxis;
            var displayId = selected.DisplayId;

            if (_scope.SelectedIndex == 2 &&
                !string.Equals(_axis.SelectedItem?.ToString(), displayId,
                    StringComparison.OrdinalIgnoreCase))
            {
                _axis.SelectedItem = displayId;
                if (_axis.SelectedIndex < 0)
                {
                    _axis.SelectedIndex = 0;
                }
            }

            _line.Enabled = _scope.SelectedIndex == 1;
            _axis.Enabled = _scope.SelectedIndex == 2;
            _scopeValue.Text = _scope.SelectedItem?.ToString() ?? "—";
            _driverValue.Text = SelectedAddress().DisplayId;
        }
        finally
        {
            _updatingSelection = false;
        }
    }

    private void OnStateChanged(object? sender, EventArgs e)
    {
        if (IsDisposed || Disposing || !IsHandleCreated)
        {
            return;
        }

        if (!InvokeRequired)
        {
            RefreshFromState();
            return;
        }

        try
        {
            BeginInvoke(new Action(RefreshFromState));
        }
        catch (ObjectDisposedException)
        {
            // Control đã được giải phóng.
        }
        catch (InvalidOperationException)
        {
            // Control đang đóng hoặc handle chưa còn hợp lệ.
        }
    }

    private void OnLogAdded(LogEntry entry)
    {
        if (IsDisposed || Disposing || !IsHandleCreated)
        {
            return;
        }

        void Append()
        {
            if (IsDisposed || Disposing)
            {
                return;
            }

            var level = entry.Level.ToString().ToUpperInvariant().PadRight(7);
            _log.AppendText(
                $"[{entry.Timestamp:HH:mm:ss}] {level} {entry.Message}" +
                Environment.NewLine);
            _log.SelectionStart = _log.TextLength;
            _log.ScrollToCaret();
        }

        if (!InvokeRequired)
        {
            Append();
            return;
        }

        try
        {
            BeginInvoke((Action)Append);
        }
        catch (ObjectDisposedException)
        {
            // Control đã được giải phóng.
        }
        catch (InvalidOperationException)
        {
            // Control đang đóng hoặc handle chưa còn hợp lệ.
        }
    }

    private void OnPageResize(object? sender, EventArgs e) => ResizeRoot();

    private void ResizeRoot()
    {
        if (_root is null || IsDisposed || Disposing)
        {
            return;
        }

        var availableWidth = Math.Max(0, ClientSize.Width - Padding.Horizontal);
        var availableHeight = Math.Max(0, ClientSize.Height - Padding.Vertical);
        var width = Math.Max(CanvasWidth - Padding.Horizontal, availableWidth);
        var height = Math.Max(CanvasHeight - Padding.Vertical, availableHeight);

        _root.Bounds = new Rectangle(Padding.Left, Padding.Top, width, height);
        AutoScrollMinSize = new Size(
            width + Padding.Horizontal,
            height + Padding.Vertical);
    }

    private async Task BusyAsync(Func<Task> action)
    {
        if (_operationRunning)
        {
            SetStatus("Một thao tác Setting khác đang chạy.", true);
            return;
        }

        _operationRunning = true;
        _save.Enabled = _read.Enabled = _reset.Enabled = false;
        try
        {
            await action();
        }
        catch (Exception ex)
        {
            _eepromValue.Text = "LỖI";
            SetStatus(ex.Message, true);
            _state.WriteLog(LogLevel.Error, $"[SETTING UI] {ex.Message}");
        }
        finally
        {
            _operationRunning = false;
            var advancedReady = _advanced is not null;
            _save.Enabled = advancedReady;
            _read.Enabled = advancedReady;
            _reset.Enabled = true;
        }
    }

    private void SetStatus(string text, bool error)
    {
        _status.Text = text;
        _status.ForeColor = error ? UiTheme.Error : UiTheme.Online;
    }

    private static TableLayoutPanel OneColumn(params float[] heights)
    {
        var panel = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = heights.Length,
            BackColor = Color.Transparent,
            Padding = new Padding(6),
            Margin = new Padding(0)
        };
        foreach (var height in heights)
            panel.RowStyles.Add(new RowStyle(SizeType.Absolute, height));
        return panel;
    }

    private static TableLayoutPanel FourColumnGrid(int rows, float titleHeight, float rowHeight)
    {
        var grid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = rows,
            BackColor = Color.Transparent,
            Padding = new Padding(6),
            Margin = new Padding(0)
        };
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 185));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 205));
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        grid.RowStyles.Add(new RowStyle(SizeType.Absolute, titleHeight));
        for (var row = 1; row < rows; row++)
            grid.RowStyles.Add(new RowStyle(SizeType.Absolute, rowHeight));
        return grid;
    }

    private static void AddPair(
        TableLayoutPanel grid,
        string label,
        Control control,
        int column,
        int row)
    {
        grid.Controls.Add(new Label
        {
            Text = label,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = UiTheme.Text,
            Font = UiTheme.FontRegular,
            BackColor = Color.Transparent
        }, column, row);

        control.Dock = DockStyle.Fill;
        control.Margin = new Padding(4, 6, 4, 6);
        grid.Controls.Add(control, column + 1, row);
    }

    private static void AddFull(TableLayoutPanel grid, Control control, int row)
    {
        control.Dock = DockStyle.Fill;
        grid.Controls.Add(control, 0, row);
        grid.SetColumnSpan(control, 4);
    }

    private static Panel Card(Padding margin) =>
        new()
        {
            Dock = DockStyle.Fill,
            BackColor = UiTheme.Surface,
            BorderStyle = BorderStyle.FixedSingle,
            Padding = new Padding(10),
            Margin = margin
        };

    private static Label Title(string text) =>
        new()
        {
            Text = text,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = UiTheme.Text,
            Font = UiTheme.FontTitle,
            BackColor = Color.Transparent
        };

    private static Label SmallLabel(string text) =>
        new()
        {
            Text = text,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.BottomLeft,
            ForeColor = UiTheme.Muted,
            Font = UiTheme.FontSmall,
            BackColor = Color.Transparent
        };

    private static Label Note(string text, Color color) =>
        new()
        {
            Text = text,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = color,
            Font = UiTheme.FontSmall,
            BackColor = Color.Transparent
        };

    private static ComboBox Combo(params string[] items)
    {
        var box = UiTheme.ComboBox();
        box.Items.AddRange(items.Cast<object>().ToArray());
        return box;
    }

    private static NumericUpDown Current(decimal value) =>
        Number(value, 0.5M, 6.0M, 0.1M, 1);

    private static NumericUpDown Number(
        decimal value,
        decimal minimum,
        decimal maximum,
        decimal increment,
        int decimals = 0)
    {
        var number = UiTheme.Numeric(value, minimum, maximum, increment, decimals);
        number.Dock = DockStyle.Fill;
        return number;
    }

    private static Label SummaryValue(string text) =>
        new()
        {
            Text = text,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            AutoEllipsis = true,
            ForeColor = UiTheme.Text,
            Font = UiTheme.FontSection,
            BackColor = Color.Transparent
        };

    private static Control SummaryCell(string label, Control value)
    {
        var panel = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            RowCount = 2,
            ColumnCount = 1,
            BackColor = UiTheme.SurfaceAlt,
            Padding = new Padding(7),
            Margin = new Padding(2)
        };
        panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 20));
        panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        panel.Controls.Add(SmallLabel(label), 0, 0);
        panel.Controls.Add(value, 0, 1);
        return panel;
    }

    private static decimal Clamp(
        decimal value,
        decimal minimum,
        decimal maximum) =>
        Math.Min(maximum, Math.Max(minimum, value));
}