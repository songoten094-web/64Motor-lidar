using WaveMotionControl.Models;
using WaveMotionControl.State;

using System.ComponentModel;

namespace WaveMotionControl.UI.Controls;

[DesignerCategory("UserControl")]
public class LogView : UserControl
{
    private readonly ApplicationState _state;
    private readonly RichTextBox _box;
    private readonly int _maxLines;
    private int _lineCount;

    public LogView() : this(new ApplicationState())
    {
    }

    public LogView(ApplicationState state, int maxLines = 400)
    {
        _state = state;
        _maxLines = maxLines;
        Dock = DockStyle.Fill;
        BackColor = UiTheme.Surface;
        Padding = new Padding(6, 4, 6, 4);

        _box = new RichTextBox
        {
            Dock = DockStyle.Fill,
            ReadOnly = true,
            BorderStyle = BorderStyle.None,
            BackColor = UiTheme.Surface,
            ForeColor = UiTheme.Text,
            Font = new Font("Consolas", 9F),
            DetectUrls = false
        };

        Controls.Add(_box);
        _state.LogAdded += OnLogAdded;
        Disposed += (_, _) => _state.LogAdded -= OnLogAdded;
    }

    private void OnLogAdded(LogEntry entry)
    {
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => OnLogAdded(entry)));
            return;
        }

        var color = entry.Level switch
        {
            LogLevel.Ok => UiTheme.Online,
            LogLevel.Warning => UiTheme.Warning,
            LogLevel.Error => UiTheme.Error,
            _ => UiTheme.Muted
        };

        _box.SelectionStart = _box.TextLength;
        _box.SelectionColor = UiTheme.Muted;
        _box.AppendText($"[{entry.Timestamp:HH:mm:ss}] ");
        _box.SelectionColor = color;
        _box.AppendText($"{entry.Level.ToString().ToUpperInvariant(),-7}");
        _box.SelectionColor = UiTheme.Text;
        _box.AppendText($" {entry.Message}{Environment.NewLine}");
        _box.ScrollToCaret();

        _lineCount++;
        if (_lineCount > _maxLines)
        {
            var text = _box.Lines.Skip(_maxLines / 4).ToArray();
            _box.Lines = text;
            _lineCount = text.Length;
            _box.SelectionStart = _box.TextLength;
            _box.ScrollToCaret();
        }
    }

    public void ClearLog()
    {
        _box.Clear();
        _lineCount = 0;
    }
}
