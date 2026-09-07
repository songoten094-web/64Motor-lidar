using WaveMotionControl.Services;
using WaveMotionControl.State;
using WaveMotionControl.UI;

namespace WaveMotionControl;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();

        var state = new ApplicationState();
        IRs485Service rs485 = new Em2RsModbusService(state);

        Application.Run(new ShellForm(state, rs485));
    }
}
