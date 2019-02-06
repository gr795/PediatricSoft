using System;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Data;
using System.ComponentModel;
using LiveCharts;
using LiveCharts.Wpf;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace PediatricSoft
{

    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        PlotWindow plotWindow = new PlotWindow();
        SendCommandsWindow sendCommandsWindow = new SendCommandsWindow();
        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private bool buttonScanPorts_IsEnabled = true;
        public bool ButtonScanPorts_IsEnabled
        {
            get { return buttonScanPorts_IsEnabled; }
            private set { buttonScanPorts_IsEnabled = value; OnPropertyChanged(); }
        }

        private bool buttonLockSensors_IsEnabled = false;
        public bool ButtonLockSensors_IsEnabled
        {
            get { return buttonLockSensors_IsEnabled; }
            private set { buttonLockSensors_IsEnabled = value; OnPropertyChanged(); }
        }

        private bool buttonRunSensors_IsEnabled = false;
        public bool ButtonRunSensors_IsEnabled
        {
            get { return buttonRunSensors_IsEnabled; }
            private set { buttonRunSensors_IsEnabled = value; OnPropertyChanged(); }
        }

        private string buttonRunSensors_Content = "Start Sensors";
        public string ButtonRunSensors_Content
        {
            get { return buttonRunSensors_Content; }
            private set { buttonRunSensors_Content = value; OnPropertyChanged(); }
        }

        private bool buttonSendCommands_IsEnabled = false;
        public bool ButtonSendCommands_IsEnabled
        {
            get { return buttonSendCommands_IsEnabled; }
            private set { buttonSendCommands_IsEnabled = value; OnPropertyChanged(); }
        }

        private bool buttonPlot_IsEnabled = false;
        public bool ButtonPlot_IsEnabled
        {
            get { return buttonPlot_IsEnabled; }
            private set { buttonPlot_IsEnabled = value; OnPropertyChanged(); }
        }

        private string buttonPlot_Content = "Show Plot";
        public string ButtonPlot_Content
        {
            get { return buttonPlot_Content; }
            private set { buttonPlot_Content = value; OnPropertyChanged(); }
        }

        private bool saveRAWValuesCheckbox_IsEnabled = false;
        public bool SaveRAWValuesCheckbox_IsEnabled
        {
            get { return saveRAWValuesCheckbox_IsEnabled; }
            private set { saveRAWValuesCheckbox_IsEnabled = value; OnPropertyChanged(); }
        }

        private bool saveDataCheckbox_IsEnabled = false;
        public bool SaveDataCheckbox_IsEnabled
        {
            get { return saveDataCheckbox_IsEnabled; }
            private set { saveDataCheckbox_IsEnabled = value; OnPropertyChanged(); }
        }

        private bool saveSuffixTextBox_IsEnabled = false;
        public bool SaveSuffixTextBox_IsEnabled
        {
            get { return saveSuffixTextBox_IsEnabled; }
            private set { saveSuffixTextBox_IsEnabled = value; OnPropertyChanged(); }
        }

        public string SaveSuffixTextBox_Text
        {
            get { return PediatricSensorData.SaveSuffix; }
            set { PediatricSensorData.SaveSuffix = value; }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public MainWindow()
        {
            InitializeComponent();
            DataContext = this;

            ThreadPool.SetMinThreads(PediatricSensorData.NumberOfThreads, PediatricSensorData.NumberOfThreads);
            plotWindow.Closed += OnPlotWindowClosing;
        }

        private async void ButtonScanPorts_Click(object sender, RoutedEventArgs e)
        {
            ButtonScanPorts_IsEnabled = false;
            ButtonLockSensors_IsEnabled = false;
            ButtonRunSensors_IsEnabled = false;
            ButtonSendCommands_IsEnabled = false;
            ButtonPlot_IsEnabled = false;
            try
            {
                plotWindow.Close();
                sendCommandsWindow.Close();
            }
            catch (Exception) { }

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Clearing sensor list\n");
            PediatricSensorData.ClearAll();

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Scanning COM ports...\n");
            await Task.Run(() => PediatricSensorData.AddAll());
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Found {PediatricSensorData.Sensors.Count} sensors\n");

            if (PediatricSensorData.Sensors.Count > 0)
            {
                if (PediatricSensorData.DebugMode)
                {
                    ButtonLockSensors_IsEnabled = true;
                    ButtonRunSensors_IsEnabled = true;
                }
                else
                {
                    ButtonLockSensors_IsEnabled = true;
                }

                ButtonSendCommands_IsEnabled = true;
                ButtonPlot_IsEnabled = true;
            }
            else
                MessageBox.Show("No sensors found. Check connection.");

            ButtonScanPorts_IsEnabled = true;

            HidePlot();
            PlotToggle(sender, e);

        }

        private async void ButtonLockSensors_Click(object sender, RoutedEventArgs e)
        {
            await Task.Run(() => PediatricSensorData.LockAll());
            ButtonRunSensors_IsEnabled = true;
        }

        private async void ButtonRunSensors_Click(object sender, RoutedEventArgs e)
        {
            PediatricSensorData.ValidateSuffixString();
            SaveSuffixTextBox_Text = PediatricSensorData.SaveSuffix;
            if (!PediatricSensorData.IsRunning)
            {
                ButtonRunSensors_IsEnabled = false;
                ButtonScanPorts_IsEnabled = false;
                SaveSuffixTextBox_IsEnabled = false;
                SaveDataCheckbox_IsEnabled = false;
                SaveRAWValuesCheckbox_IsEnabled = false;
                await Task.Run(() => PediatricSensorData.StartAll());
                ButtonRunSensors_IsEnabled = true;
                ButtonRunSensors_Content = "Stop Sensors";
            }
            else
            {
                ButtonRunSensors_IsEnabled = false;
                await Task.Run(() => PediatricSensorData.StopAll());
                ButtonScanPorts_IsEnabled = true;
                ButtonRunSensors_IsEnabled = true;
                SaveSuffixTextBox_IsEnabled = true;
                SaveDataCheckbox_IsEnabled = true;
                SaveRAWValuesCheckbox_IsEnabled = true;
                ButtonRunSensors_Content = "Start Sensors";
            }
        }

        private void ButtonPlot_Click(object sender, RoutedEventArgs e)
        {

            ButtonPlot_IsEnabled = false;
            if (plotWindow.IsVisible)
            {
                HidePlot();
            }
            else
            {
                ShowPlot();
            }
            ButtonPlot_IsEnabled = true;

        }

        private void PlotToggle(object sender, RoutedEventArgs e)
        {
            if (PediatricSensorData.SeriesCollection.Count > 0)
                PediatricSensorData.SeriesCollection.Clear();
            foreach (PediatricSensor _PediatricSensor in PediatricSensorData.Sensors)
            {
                if (_PediatricSensor.ShouldBePlotted)
                {
                    PediatricSensorData.SeriesCollection.Add(new LineSeries
                    {
                        Values = _PediatricSensor._ChartValues,
                        Fill = Brushes.Transparent,
                        PointGeometry = DefaultGeometries.None
                    });
                }
            }

        }

        private void OnPlotWindowClosing(object sender, EventArgs e)
        {
            ButtonPlot_Content = "Show Plot";
        }

        private void ShowPlot()
        {
            try
            {
                plotWindow.Show();
                plotWindow.WindowState = WindowState.Normal;
            }
            catch (Exception)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Plot window appears to be closed. Creating a new one.");
                plotWindow = new PlotWindow();
                plotWindow.Closed += OnPlotWindowClosing;
                plotWindow.Show();
            }
            ButtonPlot_Content = "Hide Plot";
        }

        private void HidePlot()
        {
            plotWindow.Hide();
            ButtonPlot_Content = "Show Plot";
        }

        private void ButtonSendCommands_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                sendCommandsWindow.Show();
                sendCommandsWindow.WindowState = WindowState.Normal;
                sendCommandsWindow.Focus();
            }
            catch (Exception)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Send commands window appears to be closed. Creating a new one.");
                sendCommandsWindow = new SendCommandsWindow();
                sendCommandsWindow.Show();
            }
        }

        private async void ButtonZeroFields_Click(object sender, RoutedEventArgs e)
        {
            await Task.Run(() => PediatricSensorData.ZeroFieldsAll());
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            PediatricSensorData.ClearAll();
            try
            {
                plotWindow.Close();
                sendCommandsWindow.Close();
            }
            catch (Exception) { }

        }


    }
}