﻿using System;
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
            UInt16ToStringBE(SensorDefaultCurrent);
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

        public const int SerialPortBaudRate = 115200;
        public const int SerialPortWriteTimeout = 1000;
        public const int SerialPortReadTimeout = 1000;
        public const int SerialPortSleepTime = 1000;
        public const int SerialPortShutDownLoopDelay = 1;
        public const int SerialPortStreamBlockSize = 4096; // 4 KiB = 32768 bits = ~284 ms at full 115200 baud
        public const int SerialPortStreamSleepMin = 150;
        public const int SerialPortStreamSleepMax = 250;

        public const int UIUpdateInterval = 500; // Update UI every X ms
        public const int StateHandlerSleepTime = 10; // in ms

        public const int ProcessingBufferSize = 1048576; // 1 MiB
        public const int DataBlockSize = 8; // Size of the data block in bytes. 8 bytes = 2 x 32 bit

        public const byte StartDataFrameByte = 0x02;
        public const byte StopDataFrameByte = 0x03;
        public const byte FrameEscapeByte = 0x10;

        public const int NumberOfThreads = 32;
        public const int MaxQueueLength = 10000;

        public const string DefaultFolder = "Data";
               
        public const string ValidIDN = "12";
        public const string CommandStringIDN = "Q0";
        public const string CommandStringSN = "Q1";
        public const string CommandStringStart = "Q2";
        public const string CommandStringStop = "Q3";

        public const string SensorCommandCurrent = "@3";
        public const ushort SensorIdleCurrent = 0x0000;
        public const ushort SensorDefaultCurrent = 0xC000;

        public const byte SensorStateInit = 0;
        public const byte SensorStateValid = 1;
        public const byte SensorStateIdle = 2;
        public const byte SensorStateLaserLock = 3;
        public const byte SensorStateRun = 4;
        public const byte SensorStateShutDown = 5;

        public const byte SensorStateLast = 255;


        // Globals
        public bool IsPlotting = false;
        public bool PlotWindowClosed = false;
        public bool SaveDataEnabled { get; set; } = false;
        public string SaveSuffix { get; set; } = String.Empty;

        public Random rnd = new Random();

        public ObservableCollection<PediatricSensor> Sensors { get; set; } = new ObservableCollection<PediatricSensor>();
        public string SensorCount { get { return String.Concat("Sensor Count: ", Sensors.Count.ToString() ); } }
        public bool IsRunning { get; private set; } = false;
        public List<SensorScanItem> SensorScanList = new List<SensorScanItem>();
        public bool IsScanning { get; private set; } = false;

        public string CommandBoxText { get; set; }

        public SeriesCollection _SeriesCollection { get; set; } = new SeriesCollection();

        public string dataFolder = String.Empty;


        public void AddAll()
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
            OnPropertyChanged("SensorCount");
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

        public string UInt16ToStringBE(ushort value)
        {
            byte[] t = BitConverter.GetBytes(value);
            Array.Reverse(t);
            string s = BitConverter.ToString(t);
            s = Regex.Replace(s, @"[^\w]", "");
            return s;
        }
    }
}