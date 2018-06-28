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
            sensorListView.ItemsSource = PediatricSensorData.Sensors;
            plotWindow.Show();
        }

        private async void ButtonScanPorts_Click(object sender, RoutedEventArgs e)
        {
            buttonScanPorts.IsEnabled = false;
            buttonRunSensors.IsEnabled = false;
            if (!PediatricSensorData.IsScanning)
            {
                if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine("Clearing sensor list\n");
                PediatricSensorData.ClearAll();

                if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine("Scanning COM ports...\n");
                await PediatricSensorData.StartScanAsync();
                if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine($"Found {PediatricSensorData.SensorScanList.Count} sensors\n");

                PediatricSensorData.AddAll();
                //if (PediatricSensorData.Sensors.Count > 0) buttonRunSensors.IsEnabled = true;
                if (PediatricSensorData.Sensors.Count > 0)
                {
                    PediatricSensorData._SeriesCollection.Clear();
                    PediatricSensorData._SeriesCollection.Add(new LineSeries
                    {
                        Values = PediatricSensorData.Sensors[0]._ChartValues,
                        Fill = Brushes.Transparent,
                        PointGeometry = DefaultGeometries.None
                    } );
                    buttonRunSensors.IsEnabled = true;
                }

            }
            buttonScanPorts.IsEnabled = true;
        }

        private void ButtonRunSensors_Click(object sender, RoutedEventArgs e)
        {
            if (!PediatricSensorData.IsRunning)
            {
                buttonRunSensors.IsEnabled = false;
                buttonScanPorts.IsEnabled = false;
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
                buttonRunSensors.Content = "Start Sensors";
            }
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            PediatricSensorData.StopAll();
            plotWindow.Close();
        }

    }
}