using System.ComponentModel;
using WaveMotionControl.Models;
using WaveMotionControl.State;

namespace WaveMotionControl.UI.Controls;

[DesignerCategory("UserControl")]
public class AxisGridControl : UserControl
{
    private readonly ApplicationState _state;
    private readonly Dictionary<AxisAddress, Button> _buttons = new();
    private AxisAddress _selectedAxis = new(1, 1);

    public AxisGridControl() : this(new ApplicationState())
    {
    }

    public AxisGridControl(ApplicationState state)
    {
        _state = state;
        Dock = DockStyle.Fill;
        BackColor = UiTheme.Surface;
        Padding = new Padding(6);

        BuildGrid();
        _state.StateChanged += OnStateChanged;
        Disposed += (_, _) => _state.StateChanged -= OnStateChanged;
    }

    public event Action<AxisAddress>? AxisSelected;

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public AxisAddress SelectedAxis
    {
        get => _selectedAxis;
        set
        {
            _selectedAxis = value;
            _state.SelectedAxis = value;
            RefreshAxisStyles();
        }
    }

    private void BuildGrid()
    {
        var table = UiTheme.Grid(17, 4);
        table.Dock = DockStyle.Fill;
        table.Padding = new Padding(1);

        table.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 46));
        for (var i = 0; i < 16; i++)
        {
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 6.25F));
        }
        for (var row = 0; row < 4; row++)
        {
            table.RowStyles.Add(new RowStyle(SizeType.Percent, 25F));
        }

        for (var line = 1; line <= 4; line++)
        {
            var lineLabel = UiTheme.Label($"LINE\n{line}", new Font("Segoe UI Semibold", 8F, FontStyle.Bold), UiTheme.Muted);
            lineLabel.Dock = DockStyle.Fill;
            lineLabel.TextAlign = ContentAlignment.MiddleCenter;
            lineLabel.AutoSize = false;
            lineLabel.BackColor = UiTheme.SurfaceAlt;
            lineLabel.Margin = new Padding(1);
            table.Controls.Add(lineLabel, 0, line - 1);

            for (var slave = 1; slave <= 16; slave++)
            {
                var address = new AxisAddress(line, slave);
                var button = new Button
                {
                    Text = address.DisplayId,
                    Dock = DockStyle.Fill,
                    Margin = new Padding(1),
                    Padding = new Padding(0),
                    FlatStyle = FlatStyle.Flat,
                    Font = new Font("Segoe UI Semibold", 7.5F, FontStyle.Bold),
                    Tag = address,
                    Cursor = Cursors.Hand
                };
                button.FlatAppearance.BorderSize = 1;
                button.Click += (_, _) =>
                {
                    SelectedAxis = address;
                    AxisSelected?.Invoke(address);
                };

                _buttons[address] = button;
                table.Controls.Add(button, slave, line - 1);
            }
        }

        Controls.Add(table);
        RefreshAxisStyles();
    }

    private void OnStateChanged(object? sender, EventArgs e)
    {
        if (InvokeRequired)
        {
            BeginInvoke(new Action(RefreshAxisStyles));
            return;
        }
        RefreshAxisStyles();
    }

    private void RefreshAxisStyles()
    {
        foreach (var pair in _buttons)
        {
            var axis = _state.GetAxis(pair.Key);
            var button = pair.Value;

            var (back, border, fore) = axis.State switch
            {
                AxisMotionState.Offline => (UiTheme.SurfaceAlt, UiTheme.Border, UiTheme.Muted),
                AxisMotionState.Online => (Color.FromArgb(6, 78, 59), UiTheme.Online, UiTheme.Text),
                AxisMotionState.Homing => (Color.FromArgb(120, 53, 15), UiTheme.Warning, UiTheme.Text),
                AxisMotionState.Homed => (Color.FromArgb(12, 74, 110), UiTheme.Homed, UiTheme.Text),
                AxisMotionState.JoggingForward or AxisMotionState.JoggingReverse or AxisMotionState.Moving =>
                    (Color.FromArgb(120, 53, 15), UiTheme.Warning, UiTheme.Text),
                AxisMotionState.Alarm => (Color.FromArgb(127, 29, 29), UiTheme.Error, UiTheme.Text),
                _ => (UiTheme.SurfaceAlt, UiTheme.Border, UiTheme.Text)
            };

            button.BackColor = back;
            button.ForeColor = fore;
            button.FlatAppearance.BorderColor = pair.Key == _selectedAxis ? UiTheme.Accent : border;
            button.FlatAppearance.BorderSize = pair.Key == _selectedAxis ? 2 : 1;
        }
    }
}
