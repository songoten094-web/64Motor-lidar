namespace WaveMotionControl.UI;

public static class UiTheme
{
    public static readonly Color Background = Color.FromArgb(11, 15, 25);    // Cyber Deep Obsidian #0B0F19
    public static readonly Color Surface = Color.FromArgb(17, 24, 39);       // Sci-Fi Dark Slate #111827
    public static readonly Color SurfaceAlt = Color.FromArgb(30, 41, 59);    // Tech Panel Slate #1E293B
    public static readonly Color Border = Color.FromArgb(51, 65, 85);        // Electric Tech Border #334155
    public static readonly Color Text = Color.FromArgb(248, 250, 252);      // Ice White #F8FAFC
    public static readonly Color Muted = Color.FromArgb(148, 163, 184);     // Cool Tech Slate #94A3B8
    public static readonly Color Accent = Color.FromArgb(0, 210, 255);      // Neon Electric Cyan #00D2FF
    public static readonly Color AccentDark = Color.FromArgb(2, 132, 199);   // Deep Electric Cyan #0284C7
    public static readonly Color Online = Color.FromArgb(16, 185, 129);     // Neon Emerald Green #10B981
    public static readonly Color Warning = Color.FromArgb(245, 158, 11);     // Cyber Amber Gold #F59E0B
    public static readonly Color Error = Color.FromArgb(239, 68, 68);       // Neon Crimson Alarm #EF4444
    public static readonly Color Homed = Color.FromArgb(56, 189, 248);      // Neon Sky Blue #38BDF8

    public static readonly Font FontRegular = new("Segoe UI", 10F, FontStyle.Regular, GraphicsUnit.Point);
    public static readonly Font FontSmall = new("Segoe UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
    public static readonly Font FontTitle = new("Segoe UI Semibold", 15F, FontStyle.Bold, GraphicsUnit.Point);
    public static readonly Font FontSection = new("Segoe UI Semibold", 11F, FontStyle.Bold, GraphicsUnit.Point);
    public static readonly Font FontMetric = new("Segoe UI Semibold", 20F, FontStyle.Bold, GraphicsUnit.Point);

    public static void Apply(Control control)
    {
        control.BackColor = Background;
        control.ForeColor = Text;
        control.Font = FontRegular;
    }

    public static Panel Card(int padding = 12)
    {
        return new Panel
        {
            BackColor = Surface,
            Padding = new Padding(padding),
            Margin = new Padding(0),
            BorderStyle = BorderStyle.FixedSingle
        };
    }

    public static Label Label(string text, Font? font = null, Color? color = null)
    {
        return new Label
        {
            Text = text,
            ForeColor = color ?? Text,
            Font = font ?? FontRegular,
            AutoSize = true,
            BackColor = Color.Transparent
        };
    }

    public static Button Button(string text, bool primary = false, bool danger = false)
    {
        var button = new Button
        {
            Text = text,
            FlatStyle = FlatStyle.Flat,
            Height = 38,
            Cursor = Cursors.Hand,
            Font = FontSection,
            ForeColor = primary ? Color.White : danger ? Error : Text,
            BackColor = primary ? AccentDark : SurfaceAlt,
            Margin = new Padding(4)
        };
        button.FlatAppearance.BorderColor = danger ? Error : primary ? Accent : Border;
        button.FlatAppearance.BorderSize = 1;
        button.FlatAppearance.MouseOverBackColor = primary ? Accent : Color.FromArgb(47, 63, 86);
        return button;
    }

    public static ComboBox ComboBox()
    {
        return new ComboBox
        {
            DropDownStyle = ComboBoxStyle.DropDownList,
            FlatStyle = FlatStyle.Flat,
            BackColor = SurfaceAlt,
            ForeColor = Text,
            Font = FontRegular,
            Height = 32,
            IntegralHeight = false
        };
    }

    public static NumericUpDown Numeric(decimal value, decimal minimum, decimal maximum, decimal increment, int decimals = 0)
    {
        var num = new NumericUpDown
        {
            Minimum = minimum,
            Maximum = maximum,
            Increment = increment,
            DecimalPlaces = decimals,
            ThousandsSeparator = true,
            BackColor = SurfaceAlt,
            ForeColor = Text,
            BorderStyle = BorderStyle.FixedSingle,
            Font = FontRegular,
            Height = 32
        };
        num.Value = Math.Clamp(value, minimum, maximum);
        return num;
    }

    public static TextBox TextBox(string text = "")
    {
        return new TextBox
        {
            Text = text,
            BackColor = SurfaceAlt,
            ForeColor = Text,
            BorderStyle = BorderStyle.FixedSingle,
            Font = FontRegular
        };
    }

    public static TableLayoutPanel Grid(int columns, int rows)
    {
        return new TableLayoutPanel
        {
            ColumnCount = columns,
            RowCount = rows,
            Dock = DockStyle.Fill,
            BackColor = Color.Transparent,
            Margin = new Padding(0),
            Padding = new Padding(0),
            GrowStyle = TableLayoutPanelGrowStyle.FixedSize
        };
    }
}
