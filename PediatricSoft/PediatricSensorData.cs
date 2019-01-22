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
        public const bool IsLaserLockDebugEnabled = false;
        public const int NumberOfThreads = 32;
        public const int DataQueueLength = 5000; // number of data points to hold in memory and plot
        public const int PlotQueueLength = 500;
        public const int UIUpdateInterval = 500; // Update UI every X ms
        public const string DefaultFolder = "Data";
        public const string ValidIDN = "12";

        public const int SerialPortBaudRate = 115200;
        public const int SerialPortWriteTimeout = 1000;
        public const int SerialPortReadTimeout = 1000;
        public const int SerialPortSleepTime = 1000;
        public const int SerialPortShutDownLoopDelay = 100;
        public const int SerialPortStreamBlockSize = 4096; // 4 KiB = 32768 bits = ~284 ms at full 115200 baud
        public const int SerialPortStreamSleepMin = 150;
        public const int SerialPortStreamSleepMax = 250;
        public const int SerialPortErrorCountMax = 10;

        public const byte StartDataFrameByte = 0x02;
        public const byte StopDataFrameByte = 0x03;
        public const byte FrameEscapeByte = 0x10;

        public const int ProcessingBufferSize = 1048576; // 1 MiB
        public const int DataBlockSize = 8; // Size of the data block in bytes. 8 bytes = 2 x 32 bit

        public const int StateHandlerSleepTime = 10; // in ms
        public const int StateHandlerCellHeatInitialTime = 5000; // 5 seconds
        public const int StateHandlerLaserHeatSweepTime = 2000; // 2 seconds
        public const int StateHandlerADCColdDelay = 1000; // 1 second

        public const double SensorTargetLaserTransmissionSweep = 0.2;
        public const double SensorTargetLaserTransmissionStep = 0.5;
        public const double SensorTargetLaserTransmissionWiggle = 0.3;

        public const int MaxNumberOfLaserLockSweepCycles = 10;
        public const int MaxNumberOfLaserLockStepCycles = 3;

        public const int SensorLaserHeatStepCycleDelay = 2000;
        public const int SensorLaserHeatStepSleepTime = 100;
        public const int SensorLaserHeatWiggleCycleDelay = 1000;
        public const int SensorLaserHeatWiggleSleepTime = 10;
        public const int SensorLaserHeatWiggleCycleLength = 10000; // in ms

        public const string SensorCommandLaserlock = "@0";
        public const ushort SensorLaserlockDisable = 0x0000;
        public const ushort SensorLaserlockEnable = 0x0001;

        public const string SensorCommandLaserCurrent = "@3";
        public const ushort SensorIdleLaserCurrent = 0x0000;
        public const ushort SensorDefaultLaserCurrent = 0xC000;

        public const string SensorCommandLaserCurrentMod = "@4";
        public const ushort SensorLaserCurrentModValue = 0x0100;

        public const string SensorCommandLaserHeat = "@5";
        public const ushort SensorIdleLaserHeat = 0x0000;
        public const ushort SensorMinLaserHeat = 0x0000;
        public const ushort SensorMaxLaserHeat = 0x2000;
        public const ushort SensorLaserHeatStep = 10;
        public const ushort SensorLaserHeatWiggleStep = 20;

        public const string SensorCommandCellHeat = "@21";
        public const ushort SensorIdleCellHeat = 0x0000;
        public const ushort SensorMinCellHeat = 0x0000;
        public const ushort SensorMaxCellHeat = 0x9000;
        public const ushort SensorCellHeatStep = 10;

        public const byte SensorStateInit = 0;
        public const byte SensorStateValid = 1;
        public const byte SensorStateIdle = 2;
        public const byte SensorStateStart = 3;
        public const byte SensorStateLaserLockSweep = 4;
        public const byte SensorStateLaserLockStep = 5;
        public const byte SensorStateLaserLockWiggle = 6;
        public const byte SensorStateLaserLockPID = 7;
        public const byte SensorStateRun = 8;
        public const byte SensorStateStop = 9;
        public const byte SensorStateFailed = 254;
        public const byte SensorStateShutDown = 255;
        


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
            Parallel.ForEach(SerialPort.GetPortNames(),port =>
            {
                PediatricSensor sensor = new PediatricSensor(port);
                sensor.Validate();
                if (sensor.IsValid)
                    App.Current.Dispatcher.Invoke((Action)delegate
                    {
                        Sensors.Add(sensor);
                    });
                else while (!sensor.Disposed) Thread.Sleep(SerialPortShutDownLoopDelay);
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
            Parallel.ForEach(Sensors, _PediatricSensor =>
            {
                _PediatricSensor.Dispose();
                while (!_PediatricSensor.Disposed) Thread.Sleep(SerialPortShutDownLoopDelay);
            });
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