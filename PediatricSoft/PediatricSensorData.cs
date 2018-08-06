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
    public sealed class PediatricSensorData : INotifyPropertyChanged
    {

        private static readonly PediatricSensorData instance = new PediatricSensorData();

        static PediatricSensorData()
        {
        }

        private PediatricSensorData()
        {
        }

        public static PediatricSensorData Instance
        {
            get
            {
                return instance;
            }
        }

        public static PediatricSensorData GetInstance() { return instance; }

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged(string prop)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop));
        }

        // Constants
        public const bool IsDebugEnabled = true;

        public readonly int DefaultSerialPortBaudRate = 115200;
        public readonly int DefaultSerialPortWriteTimeout = 1000;
        public readonly int DefaultSerialPortReadTimeout = 1000;
        public readonly int DefaultSerialPortSleepTime = 1000;
        public readonly int DefaultSerialPortShutDownLoopDelay = 1;

        public readonly int NumberOfThreads = 32;
        public readonly int MaxQueueLength = 1000;
        public readonly int GUICounter = 100;
        public readonly string DefaultFolder = "Data";

        public readonly string ValidIDN = "12";
        public readonly string CommandStringIDN = "Q0";
        public readonly string CommandStringSN = "Q1";
        public readonly string CommandStringStart = "Q2";
        public readonly string CommandStringStop = "Q3";

        // Globals
        public bool IsPlotting = false;
        public bool PlotWindowClosed = false;
        public bool SaveDataEnabled { get; set; } = false;
        public string SaveSuffix { get; set; } = String.Empty;

        public ObservableCollection<PediatricSensor> Sensors { get; set; } = new ObservableCollection<PediatricSensor>();
        public string SensorCount { get { return String.Concat("Sensor Count: ", Sensors.Count.ToString() ); } }
        public bool IsRunning { get; private set; } = false;
        public List<SensorScanItem> SensorScanList = new List<SensorScanItem>();
        public bool IsScanning { get; private set; } = false;

        public SeriesCollection _SeriesCollection { get; set; } = new SeriesCollection();

        public string dataFolder = String.Empty;


        public void AddAll()
        {
            OnPropertyChanged("SensorCount");
            Parallel.ForEach(SerialPort.GetPortNames(),port =>
            {
                PediatricSensor sensor = new PediatricSensor(port);
                sensor.Validate();
                if (sensor.IsValid)
                    App.Current.Dispatcher.Invoke((Action)delegate
                    {
                        Sensors.Add(sensor);
                    });
                OnPropertyChanged("SensorCount");
            });

        }

        public void StartAll()
        {
            if (!IsRunning)
            {
                IsRunning = true;
                if (SaveDataEnabled) CreateDataFolder();
                Parallel.ForEach(Sensors, _PediatricSensor =>
                {
                    _PediatricSensor.Start();
                });
            }
        }

        public void StopAll()
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

        public void ClearAll()
        {
            StopAll();
            Sensors.Clear();
        }

        public void ValidateSuffixString()
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

        private void CreateDataFolder()
        {
            if (String.IsNullOrEmpty(SaveSuffix))
                dataFolder = System.IO.Path.Combine(
                System.IO.Directory.GetCurrentDirectory(),
                DefaultFolder,
                DateTime.Now.ToString("yyyy-MM-dd_HHmmss"));
            else
                dataFolder = System.IO.Path.Combine(
                    System.IO.Directory.GetCurrentDirectory(),
                    DefaultFolder,
                    String.Concat(DateTime.Now.ToString("yyyy-MM-dd_HHmmss"), "_", SaveSuffix));
            System.IO.Directory.CreateDirectory(dataFolder);
        }




    }
}