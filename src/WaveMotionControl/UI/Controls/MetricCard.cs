using System.ComponentModel;

namespace WaveMotionControl.UI.Controls;

[DesignerCategory("Component")]
public class MetricCard : Panel
{
    private readonly Label _valueLabel;

    public MetricCard() : this("METRIC")
    {
    }

    public MetricCard(string title)
    {
        BackColor = UiTheme.Surface;
        BorderStyle = BorderStyle.FixedSingle;
        Margin = new Padding(4);
        Padding = new Padding(12, 8, 12, 8);

        var titleLabel = UiTheme.Label(title.ToUpperInvariant(), UiTheme.FontSmall, UiTheme.Muted);
        titleLabel.Dock = DockStyle.Top;
        titleLabel.Height = 24;

        _valueLabel = UiTheme.Label("0", UiTheme.FontMetric, UiTheme.Text);
        _valueLabel.Dock = DockStyle.Fill;
        _valueLabel.TextAlign = ContentAlignment.MiddleLeft;
        _valueLabel.AutoSize = false;

        Controls.Add(_valueLabel);
        Controls.Add(titleLabel);
    }

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public string Value
    {
        get => _valueLabel.Text;
        set => _valueLabel.Text = value;
    }

    [Browsable(false)]
    [DesignerSerializationVisibility(DesignerSerializationVisibility.Hidden)]
    public Color ValueColor
    {
        get => _valueLabel.ForeColor;
        set => _valueLabel.ForeColor = value;
    }
}
