using System.ComponentModel;
using WaveMotionControl.Models;
using WaveMotionControl.Services;
using WaveMotionControl.State;

namespace WaveMotionControl.UI.Pages;

/// <summary>
/// Panel bổ sung vào trang SETTING để đặt:
/// - Dòng HOME
/// - Dòng MANUAL
/// - Dòng AUTO
/// - Pulse/vòng dùng chung
///
/// Dòng bị khóa cứng trong khoảng 0.5–4.0 A.
/// </summary>
[DesignerCategory("Code")]
public sealed class ModeDriverSettingsPanel : UserControl
{
    private readonly ApplicationState? _state;
    private readonly IModeDriverSettingsService? _advancedService;

    private readonly TextBox _axisText;
    private readonly CheckBox _allOnline;
    private readonly NumericUpDown _homeCurrent;
    private readonly NumericUpDown _manualCurrent;
    private readonly NumericUpDown _autoCurrent;
    private readonly NumericUpDown _pulsesPerRevolution;
    private readonly Button _readButton;
    private readonly Button _saveButton;
    private readonly Label _status;

    public ModeDriverSettingsPanel()
    {
        AutoScaleMode = AutoScaleMode.Dpi;
        BackColor = UiTheme.Surface;
        ForeColor = UiTheme.Text;
        Padding = new Padding(12);
        MinimumSize = new Size(760, 300);

        _axisText = new TextBox
        {
            Text = "1.1",
            Dock = DockStyle.Fill,
            Font = new Font("Segoe UI", 10F)
        };

        _allOnline = new CheckBox
        {
            Text = "Áp dụng cho tất cả driver Online",
            AutoSize = true,
            ForeColor = ForeColor,
            Dock = DockStyle.Fill
        };
        _allOnline.CheckedChanged += (_, _) =>
            _axisText.Enabled = !_allOnline.Checked;

        _homeCurrent = CreateCurrentNumeric(3.0M);
        _manualCurrent = CreateCurrentNumeric(3.0M);
        _autoCurrent = CreateCurrentNumeric(3.0M);

        _pulsesPerRevolution = new NumericUpDown
        {
            Minimum = 200,
            Maximum = 51_200,
            Value = 10_000,
            Increment = 100,
            ThousandsSeparator = true,
            Dock = DockStyle.Fill,
            Font = new Font("Segoe UI", 10F)
        };

        _readButton = CreateButton("ĐỌC PROFILE", Color.FromArgb(51, 65, 85));
        _saveButton = CreateButton("LƯU PROFILE + GHI DRIVER", UiTheme.AccentDark);
        _readButton.Click += async (_, _) => await ReadAsync();
        _saveButton.Click += async (_, _) => await SaveAsync();

        _status = new Label
        {
            AutoSize = false,
            Dock = DockStyle.Fill,
            ForeColor = Color.Gainsboro,
            TextAlign = ContentAlignment.MiddleLeft,
            Text =
                "HOME được ghi vào 0x0191 và lưu EEPROM. MANUAL/AUTO được ghi thật " +
                "vào 0x0191 ngay trước khi chạy. Pulse/vòng dùng chung cho MOVE, AUTO " +
                "và hiển thị vị trí.",
            Font = new Font("Segoe UI", 9F)
        };

        Controls.Add(BuildLayout());
    }

    public ModeDriverSettingsPanel(
        ApplicationState state,
        IRs485Service service)
        : this()
    {
        _state = state ?? throw new ArgumentNullException(nameof(state));
        ArgumentNullException.ThrowIfNull(service);
        _advancedService = service as IModeDriverSettingsService;

        if (_advancedService is null)
        {
            Enabled = false;
            _status.Text =
                "Service hiện tại chưa hỗ trợ IModeDriverSettingsService. " +
                "Hãy dùng Em2RsModbusService.cs mới.";
            _status.ForeColor = Color.OrangeRed;
        }
    }

    private Control BuildLayout()
    {
        var root = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 4,
            RowCount = 5,
            BackColor = BackColor,
            Padding = new Padding(0),
            Margin = new Padding(0)
        };

        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 180));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 180));
        root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));

        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 58));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        var title = new Label
        {
            Text = "DÒNG THEO CHẾ ĐỘ VÀ PULSE/VÒNG",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            Font = new Font("Segoe UI Semibold", 13F, FontStyle.Bold),
            ForeColor = Color.White
        };
        root.Controls.Add(title, 0, 0);
        root.SetColumnSpan(title, 4);

        root.Controls.Add(CreateLabel("Driver (Line.Slave)"), 0, 1);
        root.Controls.Add(_axisText, 1, 1);
        root.Controls.Add(_allOnline, 2, 1);
        root.SetColumnSpan(_allOnline, 2);

        root.Controls.Add(CreateLabel("Dòng HOME (A)"), 0, 2);
        root.Controls.Add(_homeCurrent, 1, 2);
        root.Controls.Add(CreateLabel("Dòng MANUAL (A)"), 2, 2);
        root.Controls.Add(_manualCurrent, 3, 2);

        root.Controls.Add(CreateLabel("Dòng AUTO (A)"), 0, 3);
        root.Controls.Add(_autoCurrent, 1, 3);
        root.Controls.Add(CreateLabel("Pulse/vòng dùng chung"), 2, 3);
        root.Controls.Add(_pulsesPerRevolution, 3, 3);

        var buttons = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1,
            Padding = new Padding(0, 6, 0, 0)
        };
        buttons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 40));
        buttons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 60));
        buttons.Controls.Add(_readButton, 0, 0);
        buttons.Controls.Add(_saveButton, 1, 0);
        root.Controls.Add(buttons, 0, 4);
        root.SetColumnSpan(buttons, 2);

        root.Controls.Add(_status, 2, 4);
        root.SetColumnSpan(_status, 2);

        return root;
    }

    private async Task SaveAsync()
    {
        if (_state is null || _advancedService is null)
        {
            return;
        }

        AxisAddress[] targets;
        if (_allOnline.Checked)
        {
            targets = _state.Axes
                .Where(axis => axis.IsOnline)
                .Select(axis => axis.Address)
                .ToArray();

            if (targets.Length == 0)
            {
                SetStatus("Không có driver Online.", isError: true);
                return;
            }
        }
        else
        {
            if (!AxisAddress.TryParse(_axisText.Text, out var address))
            {
                SetStatus("ID không hợp lệ. Ví dụ: 1.1 đến 4.16.", isError: true);
                return;
            }

            targets = new[] { address };
        }

        var settings = new DriverModeSettings(
            (double)_homeCurrent.Value,
            (double)_manualCurrent.Value,
            (double)_autoCurrent.Value,
            (int)_pulsesPerRevolution.Value);

        await RunBusyAsync(async () =>
        {
            await _advancedService.SaveModeDriverSettingsAsync(targets, settings);
            SetStatus(
                $"Đã lưu {targets.Length} driver: HOME={settings.HomeCurrentAmps:0.0}A, " +
                $"MANUAL={settings.ManualCurrentAmps:0.0}A, " +
                $"AUTO={settings.AutoCurrentAmps:0.0}A, " +
                $"PPR={settings.PulsesPerRevolution:N0}.",
                isError: false);
        });
    }

    private async Task ReadAsync()
    {
        if (_advancedService is null)
        {
            return;
        }

        if (!AxisAddress.TryParse(_axisText.Text, out var address))
        {
            SetStatus("ID không hợp lệ. Ví dụ: 1.1 đến 4.16.", isError: true);
            return;
        }

        await RunBusyAsync(async () =>
        {
            var settings =
                await _advancedService.ReadModeDriverSettingsAsync(address);

            _homeCurrent.Value = ClampDecimal(
                (decimal)settings.HomeCurrentAmps,
                _homeCurrent.Minimum,
                _homeCurrent.Maximum);
            _manualCurrent.Value = ClampDecimal(
                (decimal)settings.ManualCurrentAmps,
                _manualCurrent.Minimum,
                _manualCurrent.Maximum);
            _autoCurrent.Value = ClampDecimal(
                (decimal)settings.AutoCurrentAmps,
                _autoCurrent.Minimum,
                _autoCurrent.Maximum);
            _pulsesPerRevolution.Value = ClampDecimal(
                settings.PulsesPerRevolution,
                _pulsesPerRevolution.Minimum,
                _pulsesPerRevolution.Maximum);

            SetStatus(
                $"Đã đọc profile {address.DisplayId}. Giới hạn dòng tối đa 6.0 A.",
                isError: false);
        });
    }

    private async Task RunBusyAsync(Func<Task> action)
    {
        _readButton.Enabled = false;
        _saveButton.Enabled = false;

        try
        {
            await action();
        }
        catch (Exception ex)
        {
            SetStatus(ex.Message, isError: true);
        }
        finally
        {
            _readButton.Enabled = true;
            _saveButton.Enabled = true;
        }
    }

    private void SetStatus(string text, bool isError)
    {
        _status.Text = text;
        _status.ForeColor = isError ? Color.OrangeRed : Color.LightGreen;
    }

    private static NumericUpDown CreateCurrentNumeric(decimal value) =>
        new()
        {
            Minimum = 0.5M,
            Maximum = 6.0M,
            Value = value,
            Increment = 0.1M,
            DecimalPlaces = 1,
            Dock = DockStyle.Fill,
            Font = new Font("Segoe UI", 10F)
        };

    private static Label CreateLabel(string text) =>
        new()
        {
            Text = text,
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Color.Gainsboro,
            Font = new Font("Segoe UI", 10F)
        };

    private static Button CreateButton(string text, Color color) =>
        new()
        {
            Text = text,
            Dock = DockStyle.Fill,
            BackColor = color,
            ForeColor = Color.White,
            FlatStyle = FlatStyle.Flat,
            Font = new Font("Segoe UI Semibold", 9F, FontStyle.Bold),
            Margin = new Padding(4)
        };

    private static decimal ClampDecimal(
        decimal value,
        decimal minimum,
        decimal maximum) =>
        Math.Min(maximum, Math.Max(minimum, value));
}
