using System.Windows;

namespace LivoxHmi.App;

public partial class App : System.Windows.Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        DispatcherUnhandledException += (_, args) =>
        {
            System.Windows.MessageBox.Show(
                args.Exception.ToString(),
                "Livox HMI - Unhandled UI exception",
                System.Windows.MessageBoxButton.OK,
                System.Windows.MessageBoxImage.Error);
            args.Handled = true;
        };

        base.OnStartup(e);

        // MainWindow is created by StartupUri. We intentionally do not perform
        // blocking work here. The window must always become visible first.
        ShutdownMode = ShutdownMode.OnMainWindowClose;
    }
}
