using System;
using System.ComponentModel;
using System.Threading;
using System.Windows;

namespace PediatricSoft
{
    public class PediatricSoftWindowManager
    {

        // Fields

        private static readonly PediatricSoftWindowManager instance = new PediatricSoftWindowManager();

        private PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;
        private DebugLog DebugLog = DebugLog.Instance;

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
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Subscribe(WindowManagerEventHandler);

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
                        if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Window Manager: Creating new Send Commands window");
                        SendCommandsWindow = new SendCommandsWindow();
                        SendCommandsWindow.Closing += SendCommandsWindowOnClosing;
                        SendCommandsWindow.Closed += SendCommandsWindowOnClosed;
                        SendCommandsWindow.Show();
                    }
                    else
                    {
                        if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Window Manager: Showing existing Send Commands window");
                        SendCommandsWindow.WindowState = WindowState.Normal;
                        SendCommandsWindow.Focus();
                    }
                    break;

                case "ShowPlotWindow":
                    if (PlotWindow == null)
                    {
                        if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Window Manager: Creating new Plot Window");
                        PlotWindow = new PlotWindow();
                        PlotWindow.Closing += PlotWindowOnClosing;
                        PlotWindow.Closed += PlotWindowOnClosed;
                        PlotWindow.Show();
                    }
                    else
                    {
                        if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Window Manager: Showing existing Plot Window");
                        PlotWindow.WindowState = WindowState.Normal;
                        PlotWindow.Focus();
                    }
                    break;

                case "ClosePlotWindow":
                    if (PlotWindow != null)
                    {
                        App.Current.Dispatcher.Invoke(() => PlotWindow.Close());
                    }
                    break;

                default:
                    break;
            }

        }

        // Event Handlers

        private void MainWindowOnClosing(object sender, CancelEventArgs e)
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Window Manager: Closing Main Window");
            if (SendCommandsWindow != null)
            {
                App.Current.Dispatcher.Invoke(() => SendCommandsWindow.Close());
            }
            if (PlotWindow != null)
            {
                App.Current.Dispatcher.Invoke(() => PlotWindow.Close());
            }
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Publish("Shutdown");
            while (!PediatricSensorData.IsDisposed) Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
            MainWindow = null;
        }

        private void SendCommandsWindowOnClosing(object sender, EventArgs e)
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Window Manager: Closing Send Commands window");
        }

        private void SendCommandsWindowOnClosed(object sender, EventArgs e)
        {
            App.Current.Dispatcher.Invoke(() => SendCommandsWindow = null);
        }

        private void PlotWindowOnClosing(object sender, EventArgs e)
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Window Manager: Closing Plot Window");
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Publish("ClearAllPlotCheckBox");
        }

        private void PlotWindowOnClosed(object sender, EventArgs e)
        {
            App.Current.Dispatcher.Invoke(() => PlotWindow = null);
        }

    }
}
