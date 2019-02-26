using Prism.Events;
using System.ComponentModel;
using System.Diagnostics;
using System.Windows;

namespace PediatricSoft
{
    public class PediatricSoftWindowManager
    {

        // Fields

        private static readonly PediatricSoftWindowManager instance = new PediatricSoftWindowManager();

        private readonly IEventAggregator eventAggregator = PediatricSoftEventGlue.eventAggregator;
        private MainWindow MainWindow;
        private SendCommandsWindow SendCommandsWindow;
        private PlotWindow PlotWindow;

        // Constructors

        static PediatricSoftWindowManager()
        {
        }

        private PediatricSoftWindowManager()
        {
        }

        // Properties

        public static PediatricSoftWindowManager Instance
        {
            get
            {
                return instance;
            }
        }

        // Methods

        public static PediatricSoftWindowManager GetInstance() { return instance; }

        public void Start()
        {
            eventAggregator.GetEvent<EventWindowManager>().Subscribe(WindowManagerEventHandler);

            MainWindow = new MainWindow();
            MainWindow.Show();
            MainWindow.Closing += MainWindowOnClosing;
            //MainWindow.Activated += WindowOnActivated;
        }

        private void WindowManagerEventHandler(string eventString)
        {
            switch (eventString)
            {
                case "ShowSendCommandsWindow":
                    if (SendCommandsWindow == null)
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Creating new Send Commands window");
                        SendCommandsWindow = new SendCommandsWindow();
                        SendCommandsWindow.Closing += SendCommandsWindowOnClosing;
                        //SendCommandsWindow.Activated += WindowOnActivated;
                        SendCommandsWindow.Show();
                    }
                    else
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Showing existing Send Commands window");
                        SendCommandsWindow.WindowState = WindowState.Normal;
                        SendCommandsWindow.Focus();
                    }
                    break;

                case "ShowPlotWindow":
                    if (PlotWindow == null)
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Creating new Plot Window");
                        PlotWindow = new PlotWindow();
                        PlotWindow.Closing += PlotWindowOnClosing;
                        //PlotWindow.Activated += WindowOnActivated;
                        PlotWindow.Show();
                    }
                    else
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Showing existing Plot Window");
                        PlotWindow.WindowState = WindowState.Normal;
                        PlotWindow.Focus();
                    }
                    break;

                case "ClosePlotWindow":
                    PlotWindow?.Close();

                    break;

                default:
                    break;
            }

        }

        // Event Handlers

        private void MainWindowOnClosing(object sender, CancelEventArgs e)
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Closing Main Window");
            SendCommandsWindow?.Close();
            PlotWindow?.Close();
            eventAggregator.GetEvent<EventDataLayer>().Publish("Shutdown");
            MainWindow = null;
        }

        private void SendCommandsWindowOnClosing(object sender, CancelEventArgs e)
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Closing Send Commands window");
            SendCommandsWindow = null;
        }

        private void PlotWindowOnClosing(object sender, CancelEventArgs e)
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Closing Plot Window");
            PlotWindow = null;
            eventAggregator.GetEvent<EventDataLayer>().Publish("ClearAllPlotCheckBox");
        }

        private void WindowOnActivated(object sender, System.EventArgs e)
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Window Manager: Bringing up all windows");
            Window window = (Window)sender;

            window.Activated -= WindowOnActivated;

            if (MainWindow != null && MainWindow != window)
            {
                MainWindow.WindowState = WindowState.Normal;
                MainWindow.Focus();
            }
            if (SendCommandsWindow != null && SendCommandsWindow != window)
            {
                SendCommandsWindow.WindowState = WindowState.Normal;
                SendCommandsWindow.Focus();
            }
            if (PlotWindow != null && PlotWindow != window)
            {
                PlotWindow.WindowState = WindowState.Normal;
                PlotWindow.Focus();
            }

            window.WindowState = WindowState.Normal;
            window.Focus();

            window.Activated += WindowOnActivated;
        }

    }
}
