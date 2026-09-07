#nullable enable
using System.ComponentModel;

namespace WaveMotionControl.UI.Pages;

public partial class ManualPage
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
        // ManualPage
        // 
        AutoScaleDimensions = new SizeF(120F, 120F);
        AutoScaleMode = AutoScaleMode.Dpi;
        BackColor = Color.FromArgb(11, 15, 25);
        Name = "ManualPage";
        Size = new Size(1357, 620);
        Load += ManualPage_Load;
        ResumeLayout(false);
    }
}
