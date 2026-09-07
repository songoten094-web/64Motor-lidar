#nullable enable
using System.ComponentModel;

namespace WaveMotionControl.UI.Pages;

public partial class AutoPage
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
        // AutoPage
        // 
        AutoScaleDimensions = new SizeF(120F, 120F);
        AutoScaleMode = AutoScaleMode.Dpi;
        BackColor = Color.FromArgb(11, 15, 25);
        Name = "AutoPage";
        Size = new Size(1357, 620);
        Load += AutoPage_Load_1;
        ResumeLayout(false);
    }
}
