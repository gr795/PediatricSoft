﻿using System;
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

namespace PediatricSoft
{

    public partial class MainWindow : Window
    {

        PlotWindow plotWindow = new PlotWindow();
        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        public MainWindow()
        {
            InitializeComponent();
            ThreadPool.SetMinThreads(PediatricSensorData.NumberOfThreads, PediatricSensorData.NumberOfThreads);

            plotWindow.Closed += OnPlotWindowClosing;
        }

        private async void ButtonScanPorts_Click(object sender, RoutedEventArgs e)
        {
            buttonScanPorts.IsEnabled = false;
            buttonRunSensors.IsEnabled = false;
            buttonPlot.IsEnabled = false;

            if (!PediatricSensorData.IsScanning)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Clearing sensor list\n");
                PediatricSensorData.ClearAll();

                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Scanning COM ports...\n");
                await Task.Run(() => PediatricSensorData.AddAll());
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Found {PediatricSensorData.Sensors.Count} sensors\n");

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

        private async void ButtonRunSensors_Click(object sender, RoutedEventArgs e)
        {
            PediatricSensorData.ValidateSuffixString();
            saveSuffixTextBox.Text = PediatricSensorData.SaveSuffix;
            if (!PediatricSensorData.IsRunning)
            {
                buttonRunSensors.IsEnabled = false;
                buttonScanPorts.IsEnabled = false;
                saveSuffixTextBox.IsEnabled = false;
                saveDataCheckbox.IsEnabled = false;
                await Task.Run(() => PediatricSensorData.StartAll());
                buttonRunSensors.IsEnabled = true;
                buttonRunSensors.Content = "Stop Sensors";
            }
            else
            {
                buttonRunSensors.IsEnabled = false;
                await Task.Run(() => PediatricSensorData.StopAll());
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
            if (plotWindow.IsVisible)
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

        private void OnPlotWindowClosing(object sender, EventArgs e)
        {
            buttonPlot.Content = "Show Plot";
        }

        private void ShowPlot()
        {
            try
            {
                plotWindow.Show();
            }
            catch (Exception)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Plot window appears to be closed. Creating a new one.");
                plotWindow = new PlotWindow();
                plotWindow.Closed += OnPlotWindowClosing;
                plotWindow.Show();
            }
            buttonPlot.Content = "Hide Plot";
        }

        private void HidePlot()
        {
            plotWindow.Hide();
            buttonPlot.Content = "Show Plot";
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            PediatricSensorData.StopAll();
            try
            {
                plotWindow.Close();
            }
            catch (Exception) { }

        }
    }
}