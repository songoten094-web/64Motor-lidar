#nullable enable
using System.ComponentModel;

namespace WaveMotionControl.UI.Pages;

public partial class MainPage
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
        // MainPage
        // 
        AutoScaleDimensions = new SizeF(120F, 120F);
        AutoScaleMode = AutoScaleMode.Dpi;
        BackColor = Color.FromArgb(11, 15, 25);
        Name = "MainPage";
        Size = new Size(1357, 620);
        Load += MainPage_Load;
        ResumeLayout(false);
    }
}
