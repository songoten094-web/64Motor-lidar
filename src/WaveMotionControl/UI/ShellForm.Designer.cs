#nullable enable
using System.ComponentModel;

namespace WaveMotionControl.UI;

public partial class ShellForm
{
    private IContainer? components = null;

    protected override void Dispose(bool disposing)
    {
        if (disposing && (components != null))
        {
            components.Dispose();
        }
        base.Dispose(disposing);
    }

    private void InitializeComponent()
    {
        SuspendLayout();
        // 
        // ShellForm
        // 
        AutoScaleDimensions = new SizeF(120F, 120F);
        AutoScaleMode = AutoScaleMode.Dpi;
        BackColor = Color.FromArgb(11, 15, 25);
        ClientSize = new Size(1920, 1055);
        Font = new Font("Segoe UI", 9.5F);
        ForeColor = Color.FromArgb(248, 250, 252);
        KeyPreview = true;
        MinimumSize = new Size(1280, 720);
        Name = "ShellForm";
        StartPosition = FormStartPosition.CenterScreen;
        Text = "Wave Motion Control — 64 EM2RS Drivers";
        WindowState = FormWindowState.Maximized;
        Load += ShellForm_Load;
        ResumeLayout(false);
    }
}
