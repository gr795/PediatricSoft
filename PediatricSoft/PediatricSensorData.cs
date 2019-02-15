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
using System.Runtime.CompilerServices;
using Prism.Mvvm;
using Prism.Commands;
using System.Windows.Media;
using System.Windows;
using Prism.Events;
using FTD2XX_NET;

namespace PediatricSoft
{
    public sealed class PediatricSensorData : BindableBase, IDisposable
    {

        private static readonly PediatricSensorData instance = new PediatricSensorData();

        private readonly IEventAggregator eventAggregator = PediatricSoftEventGlue.eventAggregator;

        static PediatricSensorData()
        {
        }

        private PediatricSensorData()
        {
            eventAggregator.GetEvent<EventDataLayer>().Subscribe(DataLayerEventHandler);
        }

        public static PediatricSensorData Instance
        {
            get
            {
                return instance;
            }
        }

        public static PediatricSensorData GetInstance() { return instance; }

        // Constants
        public const bool IsDebugEnabled = true;
        public const bool IsLaserLockDebugEnabled = false;
        public const int NumberOfThreads = 128;
        public const int DataQueueLength = 5000; // number of data points to hold in memory and plot
        public const int DataQueueRunningAvgLength = 10; // number of data points for the running average
        public const int PlotQueueLength = 500;
        public const int UIUpdateInterval = 250; // Update UI every X ms
        public const string ValidIDN = "Arrow USB Blaster B";

        public const UInt32 SerialPortBaudRate = 115200;
        public const UInt32 SerialPortWriteTimeout = 250;
        public const UInt32 SerialPortReadTimeout = 250;
        public const int SerialPortSleepTime = 100;
        public const int SerialPortShutDownLoopDelay = 100;
        public const UInt32 SerialPortStreamBlockSize = 16;
        public const int SerialPortStreamSleepMin = 1;
        public const int SerialPortStreamSleepMax = 1;
        public const int SerialPortErrorCountMax = 10;

        public const byte StartDataFrameByte = 0x02;
        public const byte StopDataFrameByte = 0x03;
        public const byte StartInfoFrameByte = 0x3F;
        public const byte FrameEscapeByte = 0x10;

        public const UInt32 ProcessingBufferSize = 1048576; // 1 MiB
        public const UInt32 DataBlockSize = 8; // Size of the data block in bytes. 8 bytes = 2 x 32 bit
        public const UInt32 InfoBlockSize = 4;

        public const int StateHandlerSleepTime = 10; // in ms
        public const int StateHandlerCellHeatInitialTime = 5000; // 5 seconds
        public const int StateHandlerLaserHeatSweepTime = 1000; // 1 second
        public const int StateHandlerADCColdDelay = 1000; // 1 second

        public const double SensorTargetLaserTransmissionSweep = 0.2;
        public const double SensorTargetLaserTransmissionStep = 0.5;

        public const double SensorADCRawToVolts = (double)5 / 125 / 16777215;
        public const double SensorADCColdValueLowGainMinVolts = 0.5; // Minimum ADC voltage on low gain

        public const double SensorCoilsCalibrationTeslaPerHex = 21e-12; // 21 pT per step

        public const int MaxNumberOfLaserLockSweepCycles = 30;
        public const int MaxNumberOfLaserLockStepCycles = 3;
        public const int NumberOfFieldZeroingSteps = 100;
        public const int NumberOfMagnetometerCalibrationSteps = 100;

        public const int SensorLaserHeatStepCycleDelay = 2000;
        public const int SensorLaserHeatStepSleepTime = 100;

        public const string SensorCommandLaserlock = "@0";
        public const ushort SensorLaserlockDisable = 0x0000;
        public const ushort SensorLaserlockEnable = 0x0005;

        public const string SensorCommandLaserCurrent = "@3";
        public const ushort SensorColdLaserCurrent = 0x0000;
        public const ushort SensorDefaultLaserCurrent = 0x9000;

        public const string SensorCommandLaserCurrentMod = "@4";
        public const ushort SensorLaserCurrentModValue = 0x0250;

        public const string SensorCommandLaserHeat = "@5";
        public const ushort SensorColdLaserHeat = 0x0000;
        public const ushort SensorDefaultLaserHeat = 0x0500;
        public const ushort SensorMinLaserHeat = 0x0000;
        public const ushort SensorMaxLaserHeat = 0x2000;
        public const ushort SensorLaserHeatStep = 10;

        public const string SensorCommandFieldXOffset = "@7";
        public const ushort SensorColdFieldXOffset = 0x8000;

        public const string SensorCommandFieldXAmplitude = "@8";
        public const ushort SensorColdFieldXAmplitude = 0x0000;

        public const string SensorCommandFieldYOffset = "@9";
        public const ushort SensorColdFieldYOffset = 0x8000;

        public const string SensorCommandFieldYAmplitude = "@A";
        public const ushort SensorColdFieldYAmplitude = 0x0000;

        public const string SensorCommandFieldZOffset = "@B";
        public const ushort SensorColdFieldZOffset = 0x8000;
        public const ushort SensorLaserLockFieldZOffset = 0x0000;

        public const string SensorCommandFieldZAmplitude = "@C";
        public const ushort SensorColdFieldZAmplitude = 0x0000;
        public const ushort SensorRunFieldZAmplitude = 0x07C0;

        public const ushort SensorFieldCheckRange = 0x00EE; // about 5 nT assuming 21 pT per hex step
        public const ushort SensorFieldStep = 0x0018; // about 0.5 nT assuming 21 pT per hex step

        public const string SensorCommandDigitalDataStreamingAndGain = "@20";
        public const ushort SensorDigitalDataStreamingOffGainLow = 0x0000;
        public const ushort SensorDigitalDataStreamingOffGainHigh = 0x0001;
        public const ushort SensorDigitalDataStreamingOnGainLow = 0x0002;
        public const ushort SensorDigitalDataStreamingOnGainHigh = 0x0003;

        public const string SensorCommandCellHeat = "@21";
        public const ushort SensorColdCellHeat = 0x0000;
        public const ushort SensorRunCellHeat = 0x5000;
        public const ushort SensorMinCellHeat = 0x0000;
        public const ushort SensorMaxCellHeat = 0x9000;
        public const ushort SensorCellHeatStep = 10;

        public const string SensorCommandDigitalDataSelector = "@22";
        public const ushort SensorDigitalDataSelectorADC = 0x0000;
        public const ushort SensorDigitalDataSelectorZDemod = 0x0005;

        public const string SensorCommandGetOptimalLaserCurrent = "@2C";
        public const string SensorCommandGetOptimalBzMod = "@2D";
        public const string SensorCommandGetOptimalCellHeatLockPoint = "@2E";
        public const string SensorCommandGetMaxCellHeat = "@2F"; // Not used. Set to 9000

        public const byte SensorStateInit = 0;
        public const byte SensorStateValid = 1;
        public const byte SensorStateGetOptimalParameters = 2;
        public const byte SensorStateMakeCold = 3;
        public const byte SensorStateCold = 4;
        public const byte SensorStateLockStart = 5;
        public const byte SensorStateLaserLockSweep = 6;
        public const byte SensorStateLaserLockStep = 7;
        public const byte SensorStateLaserLockPID = 8;
        public const byte SensorStateStabilizeCellHeat = 9;
        public const byte SensorStateZeroFields = 10;
        public const byte SensorStateCalibrateMagnetometer = 11;
        public const byte SensorStateIdle = 12;
        public const byte SensorStateStart = 13;
        public const byte SensorStateRun = 14;
        public const byte SensorStateStop = 15;
        public const byte SensorStateFailed = 254;
        public const byte SensorStateShutDown = 255;

        // Globals
        public bool DebugMode { get; set; } = false;
        public bool SaveDataEnabled { get; set; } = false;
        public bool SaveRAWValues { get; set; } = false;
        public string SaveFolder { get; set; } = String.Empty;
        public string SaveFolderCurrentRun { get; set; } = String.Empty;
        public string SaveSuffix { get; set; } = String.Empty;
        public string CommandHistory { get; set; } = String.Empty;

        public ObservableCollection<PediatricSensor> Sensors { get; private set; } = new ObservableCollection<PediatricSensor>();
        public SeriesCollection SeriesCollection { get; private set; } = new SeriesCollection();

        public int SensorCount { get { return Sensors.Count; } }

        private bool isRunning = false;
        public bool IsRunning
        {
            get { return isRunning; }
            private set { isRunning = value; RaisePropertyChanged(); }
        }

        private bool canSendCommands = false;
        public bool CanSendCommands
        {
            get { return canSendCommands; }
            private set { canSendCommands = value; RaisePropertyChanged(); }
        }

        private bool canScan = true;
        public bool CanScan
        {
            get { return canScan; }
            private set { canScan = value; RaisePropertyChanged(); }
        }
        public bool ScanPortsAsyncCanExecute() { return CanScan; }

        public void ScanPortsAsync()
        {
            Task.Run(() =>
            {
                if (CanScan)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Begin port scan");

                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    Parallel.ForEach(Sensors, sensor =>
                    {
                        if (sensor.State == PediatricSensorData.SensorStateFailed)
                            App.Current.Dispatcher.Invoke(() => Sensors.Remove(sensor));
                    });

                    Parallel.ForEach(GetPotentialSensorSerialNumbers(), serial =>
                     {
                         PediatricSensor sensor = new PediatricSensor(serial);
                         if (sensor.IsValid)
                         {
                             sensor.KickOffTasks();
                             App.Current.Dispatcher.Invoke(() => Sensors.Add(sensor));
                         }
                         else
                             while (!sensor.IsDisposed)
                                 Thread.Sleep(StateHandlerSleepTime);
                         RaisePropertyChanged("SensorCount");
                     });

                    if (Sensors.Count > 0)
                    {
                        CanLock = true;
                        CanSendCommands = true;
                    }

                    CanScan = true;

                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Port scan done");
                }
            });
        }

        private bool canLock = false;
        public bool CanLock
        {
            get { return canLock; }
            set { canLock = value; RaisePropertyChanged(); }
        }
        public bool LockAllAsyncCanExecute() { return CanLock; }

        public void LockAllAsync()
        {
            Task.Run(() =>
            {
                if (CanLock)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Begin sensor lock");

                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;


                    Parallel.ForEach(Sensors, sensor =>
                    {
                        sensor.Lock();
                    });

                    CanScan = true;
                    CanLock = true;
                    CanStartStop = true;
                    CanZeroFields = true;
                    CanSendCommands = true;

                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Sensor lock done");
                }
            });
        }

        private bool canStartStop = false;
        public bool CanStartStop
        {
            get { return canStartStop; }
            set { canStartStop = value; RaisePropertyChanged(); }
        }
        public bool StartStopAsyncCanExecute() { return CanStartStop; }

        public void StartStopAsync()
        {
            Task.Run(() =>
            {
                if (CanStartStop)
                {
                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    if (IsRunning)
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Stopping all sensors");

                        Parallel.ForEach(Sensors, sensor =>
                        {
                            sensor.Stop();
                        });

                        IsRunning = false;

                        CanScan = true;
                        CanLock = true;
                        CanZeroFields = true;
                        CanSendCommands = true;
                    }
                    else
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Starting all sensors");

                        IsRunning = true;

                        if (SaveDataEnabled) CreateDataFolder();
                        Parallel.ForEach(Sensors, sensor =>
                        {
                            sensor.Start();
                        });
                    }

                    CanStartStop = true;
                }
            });
        }

        private bool canZeroFields = false;
        public bool CanZeroFields
        {
            get { return canZeroFields; }
            set { canZeroFields = value; RaisePropertyChanged(); }
        }
        public bool ZeroFieldsAsyncCanExecute() { return CanZeroFields; }

        public void ZeroFieldsAsync()
        {
            Task.Run(() =>
            {
                if (CanZeroFields)
                {
                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    Parallel.ForEach(Sensors, sensor =>
                    {
                        sensor.ZeroFields();
                    });

                    CanScan = true;
                    CanLock = true;
                    CanStartStop = true;
                    CanZeroFields = true;
                    CanSendCommands = true;
                }
            });
        }

        public void UpdateSeriesCollection()
        {
            try { SeriesCollection.Clear(); }
            catch (Exception) { }

            int count = 0;

            Parallel.ForEach(Sensors, sensor =>
            {
                if (sensor.IsPlotted)
                {
                    App.Current.Dispatcher.Invoke(() =>
                    {
                        SeriesCollection.Add(new LineSeries
                        {
                            Values = sensor.ChartValues,
                            Fill = Brushes.Transparent,
                            PointGeometry = DefaultGeometries.None
                        });
                    });
                    App.Current.Dispatcher.Invoke(() =>
                    {
                        count++;
                    });
                }

            });

            if (count > 0)
                eventAggregator.GetEvent<EventWindowManager>().Publish("ShowPlotWindow");
            else
                eventAggregator.GetEvent<EventWindowManager>().Publish("ClosePlotWindow");

        }

        private string[] GetPotentialSensorSerialNumbers()
        {
            List<string> SerialNumbers = new List<string>();

            UInt32 ftdiDeviceCount = 0;
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            // Create new instance of the FTDI device class
            FTDI myFtdiDevice = new FTDI();

            // Determine the number of FTDI devices connected to the machine
            ftStatus = myFtdiDevice.GetNumberOfDevices(ref ftdiDeviceCount);

            // Check status
            if (ftStatus == FTDI.FT_STATUS.FT_OK)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Number of FTDI devices: " + ftdiDeviceCount.ToString());
            }
            else
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Failed to get number of devices (error " + ftStatus.ToString() + ")");
                return SerialNumbers.ToArray();
            }

            // If no devices available, return
            if (ftdiDeviceCount == 0)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "FTDI devices found");
                return SerialNumbers.ToArray();
            }

            // Allocate storage for device info list
            FTDI.FT_DEVICE_INFO_NODE[] ftdiDeviceList = new FTDI.FT_DEVICE_INFO_NODE[ftdiDeviceCount];

            // Populate our device list
            ftStatus = myFtdiDevice.GetDeviceList(ftdiDeviceList);

            // Add potential sensors to the list
            if (ftStatus == FTDI.FT_STATUS.FT_OK)
            {
                for (UInt32 i = 0; i < ftdiDeviceCount; i++)
                {
                    if (ftdiDeviceList[i].Description.ToString() == PediatricSensorData.ValidIDN)
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, "Adding a potential sensor with S/N: " + ftdiDeviceList[i].SerialNumber.ToString());
                        SerialNumbers.Add(ftdiDeviceList[i].SerialNumber.ToString());
                    }
                }
            }

            return SerialNumbers.ToArray();
        }




        private void DataLayerEventHandler(string eventString)
        {
            ClearAll();
        }

        public void ClearAll()
        {
            Parallel.ForEach(Sensors, _PediatricSensor =>
            {
                _PediatricSensor.Dispose();
                while (!_PediatricSensor.IsDisposed) Thread.Sleep(StateHandlerSleepTime);
            });
            Sensors.Clear();
            RaisePropertyChanged("SensorCount");
        }

        private void CreateDataFolder()
        {
            if (String.IsNullOrEmpty(SaveSuffix))
                SaveFolderCurrentRun = System.IO.Path.Combine(
                SaveFolder,
                DateTime.Now.ToString("yyyy-MM-dd_HHmmss"));
            else
                SaveFolderCurrentRun = System.IO.Path.Combine(
                    SaveFolder,
                    String.Concat(DateTime.Now.ToString("yyyy-MM-dd_HHmmss"), "_", Regex.Replace(SaveSuffix, @"[^\w]", "")));
            System.IO.Directory.CreateDirectory(SaveFolderCurrentRun);
        }

        public void Dispose()
        {
            Parallel.ForEach(Sensors, sensor =>
            {
                sensor.Dispose();
            });
        }

    }
}