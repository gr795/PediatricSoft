using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.ComponentModel;
using LiveCharts;
using LiveCharts.Wpf;


namespace PediatricSoft
{

    public partial class MainWindow : Window
    {

        PlotWindow plotWindow = new PlotWindow();

        public MainWindow()
        {
            InitializeComponent();
        }

        private async void ButtonScanPorts_Click(object sender, RoutedEventArgs e)
        {
            buttonScanPorts.IsEnabled = false;
            buttonRunSensors.IsEnabled = false;
            buttonPlot.IsEnabled = false;

            if (!PediatricSensorData.IsScanning)
            {
                if (PediatricSensorData.IsDebugEnabled) Console.WriteLine("Clearing sensor list\n");
                PediatricSensorData.ClearAll();

                if (PediatricSensorData.IsDebugEnabled) Console.WriteLine("Scanning COM ports...\n");
                await PediatricSensorData.StartScanAsync();
                if (PediatricSensorData.IsDebugEnabled) Console.WriteLine($"Found {PediatricSensorData.SensorScanList.Count} sensors\n");

                PediatricSensorData.AddAll();
                if (PediatricSensorData.Sensors.Count > 0)
                {
                    buttonRunSensors.IsEnabled = true;
                    buttonPlot.IsEnabled = true;
                }

            }
            buttonScanPorts.IsEnabled = true;

            HidePlot();
            PlotToggle(sender, e);

        }

        private void ButtonRunSensors_Click(object sender, RoutedEventArgs e)
        {
            PediatricSensorData.ValidatePrefixString();
            saveSuffixTextBox.Text = PediatricSensorData.SaveSuffix;
            if (!PediatricSensorData.IsRunning)
            {
                buttonRunSensors.IsEnabled = false;
                buttonScanPorts.IsEnabled = false;
                saveSuffixTextBox.IsEnabled = false;
                saveDataCheckbox.IsEnabled = false;
                PediatricSensorData.StartAll();
                buttonRunSensors.IsEnabled = true;
                buttonRunSensors.Content = "Stop Sensors";
            }
            else
            {
                buttonRunSensors.IsEnabled = false;
                PediatricSensorData.StopAll();
                buttonScanPorts.IsEnabled = true;
                buttonRunSensors.IsEnabled = true;
                saveSuffixTextBox.IsEnabled = true;
                saveDataCheckbox.IsEnabled = true;
                buttonRunSensors.Content = "Start Sensors";
            }
        }

        private void ButtonPlot_Click(object sender, RoutedEventArgs e)
        {

            buttonPlot.IsEnabled = false;
            if (PediatricSensorData.IsPlotting)
            {
                HidePlot();
            }
            else
            {
                ShowPlot();
            }
            buttonPlot.IsEnabled = true;

        }

        private void PlotToggle(object sender, RoutedEventArgs e)
        {
            PediatricSensorData._SeriesCollection.Clear();
            foreach (PediatricSensor _PediatricSensor in PediatricSensorData.Sensors)
            {
                if (_PediatricSensor.ShouldBePlotted)
                {
                    PediatricSensorData._SeriesCollection.Add(new LineSeries
                    {
                        Values = _PediatricSensor._ChartValues,
                        Fill = Brushes.Transparent,
                        PointGeometry = DefaultGeometries.None
                    });
                }
            }

        }

        private void ShowPlot()
        {
            if (PediatricSensorData.PlotWindowClosed)
            {
                plotWindow = new PlotWindow();
                PediatricSensorData.PlotWindowClosed = false;
            }
            plotWindow.Show();
            PediatricSensorData.IsPlotting = true;
            buttonPlot.Content = "Hide Plot";
        }

        public void HidePlot()
        {
            plotWindow.Hide();
            PediatricSensorData.IsPlotting = false;
            buttonPlot.Content = "Show Plot";
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            PediatricSensorData.StopAll();
            plotWindow.Close();
        }

    }
}