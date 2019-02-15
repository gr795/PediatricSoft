﻿using Prism.Events;
using System.ComponentModel;
using System.Diagnostics;
using System.Windows;

namespace PediatricSoft
{
    public class PediatricSoftWindowManager
    {

        // Fields

        private static readonly PediatricSoftWindowManager instance = new PediatricSoftWindowManager();

        private readonly IEventAggregator eventAggregator;
        private MainWindow MainWindow;
        private SendCommandsWindow SendCommandsWindow;
        private PlotWindow PlotWindow;

        // Constructors

        static PediatricSoftWindowManager()
        {
        }

        private PediatricSoftWindowManager()
        {
            eventAggregator = PediatricSoftEventGlue.eventAggregator;
            eventAggregator.GetEvent<EventWindowManager>().Subscribe(WindowManagerEventHandler);

            MainWindow = new MainWindow();
            MainWindow.Show();
            MainWindow.Closing += MainWindowOnClosing;
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

        private void WindowManagerEventHandler(string eventString)
        {
            switch (eventString)
            {
                case "ShowSendCommandsWindow":
                    if (SendCommandsWindow == null)
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Window Manager: Creating new Send Commands window");
                        SendCommandsWindow = new SendCommandsWindow();
                        SendCommandsWindow.Closing += SendCommandsWindowOnClosing;
                        SendCommandsWindow.Show();
                    }
                    else
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Window Manager: Showing existing Send Commands window");
                        SendCommandsWindow.WindowState = WindowState.Normal;
                        SendCommandsWindow.Focus();
                    }
                    break;

                case "ShowPlotWindow":
                    if (PlotWindow == null)
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Window Manager: Creating new Plot Window");
                        PlotWindow = new PlotWindow();
                        PlotWindow.Closing += PlotWindowOnClosing;
                        PlotWindow.Show();
                    }
                    else
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Window Manager: Showing existing Plot Window");
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
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Window Manager: Closing Main Window");
            SendCommandsWindow?.Close();
            PlotWindow?.Close();
            App.Current.Dispatcher.Invoke(() => eventAggregator.GetEvent<EventDataLayer>().Publish("Shutdown"));
            MainWindow = null;
        }

        private void SendCommandsWindowOnClosing(object sender, CancelEventArgs e)
        {
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Window Manager: Closing Send Commands window");
            SendCommandsWindow = null;
        }

        private void PlotWindowOnClosing(object sender, CancelEventArgs e)
        {
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Window Manager: Closing Plot Window");
            PlotWindow = null;
        }

    }
}
