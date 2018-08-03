using System;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Text.RegularExpressions;
using LiveCharts;
using LiveCharts.Wpf;
using System.Diagnostics;
using System.ComponentModel;

namespace PediatricSoft
{
    public class PediatricSensorData
    {

        // Constants
        public const bool IsDebugEnabled = true;

        public static readonly int DefaultSerialPortBaudRate = 115200;
        public static readonly int DefaultSerialPortWriteTimeout = 1000;
        public static readonly int DefaultSerialPortReadTimeout = 1000;
        public static readonly int DefaultSerialPortSleepTime = 1000;
        public static readonly int DefaultSerialPortShutDownLoopDelay = 1;

        public static readonly int NumberOfThreads = 32;
        public static readonly int MaxQueueLength = 1000;
        public static readonly int GUICounter = 100;
        public static readonly string DefaultFolder = "Data";

        public static readonly string ValidIDN = "12";
        public static readonly string CommandStringIDN = "Q0";
        public static readonly string CommandStringSN = "Q1";
        public static readonly string CommandStringStart = "Q2";
        public static readonly string CommandStringStop = "Q3";

        // Globals
        public static bool IsPlotting = false;
        public static bool PlotWindowClosed = false;
        public static bool SaveDataEnabled { get; set; } = false;
        public static string SaveSuffix { get; set; } = String.Empty;

        public static ObservableCollection<PediatricSensor> Sensors { get; set; } = new ObservableCollection<PediatricSensor>();
        public static string SensorCount { get { return Sensors.Count.ToString(); } }
        public static bool IsRunning { get; private set; } = false;
        public static List<SensorScanItem> SensorScanList = new List<SensorScanItem>();
        public static bool IsScanning { get; private set; } = false;

        public static SeriesCollection _SeriesCollection { get; set; } = new SeriesCollection();

        public static string dataFolder = String.Empty;

        static PediatricSensorData()
        {

        }

        public static void AddAll()
        {

            Parallel.ForEach(SerialPort.GetPortNames(),port =>
            {
                PediatricSensor sensor = new PediatricSensor(port);
                sensor.Validate();
                if (sensor.IsValid)
                    App.Current.Dispatcher.Invoke((Action)delegate
                    {
                        Sensors.Add(sensor);
                    });
                //OnPropertyChanged("SensorCount");
            });

        }

        public static void StartAll()
        {
            if (!IsRunning)
            {
                IsRunning = true;
                if (PediatricSensorData.SaveDataEnabled) CreateDataFolder();
                Parallel.ForEach(Sensors, _PediatricSensor =>
                {
                    _PediatricSensor.Start();
                });
            }
        }

        public static void StopAll()
        {
            if (IsRunning)
            {
                Parallel.ForEach(Sensors, _PediatricSensor =>
                {
                    _PediatricSensor.Stop();
                });
                IsRunning = false;
            }
        }

        public static void ClearAll()
        {
            StopAll();
            Sensors.Clear();
        }

        public static void ValidateSuffixString()
        {
            // Replace invalid characters with empty strings.
            try
            {
                SaveSuffix = Regex.Replace(SaveSuffix, @"[^\w]", "",
                                     RegexOptions.None, TimeSpan.FromSeconds(1.5));
            }
            // If we timeout when replacing invalid characters, 
            // we should return Empty.
            catch (RegexMatchTimeoutException)
            {
                SaveSuffix = String.Empty;
            }
        }

        private static void CreateDataFolder()
        {
            if (String.IsNullOrEmpty(SaveSuffix))
                dataFolder = System.IO.Path.Combine(
                System.IO.Directory.GetCurrentDirectory(),
                PediatricSensorData.DefaultFolder,
                DateTime.Now.ToString("yyyy-MM-dd_HHmmss"));
            else
                dataFolder = System.IO.Path.Combine(
                    System.IO.Directory.GetCurrentDirectory(),
                    PediatricSensorData.DefaultFolder,
                    String.Concat(DateTime.Now.ToString("yyyy-MM-dd_HHmmss"), "_", SaveSuffix));
            System.IO.Directory.CreateDirectory(dataFolder);
        }




    }
}