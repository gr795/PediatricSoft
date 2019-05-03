using FTD2XX_NET;
using Prism.Mvvm;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace PediatricSoft
{
    public sealed class PediatricSensorData : BindableBase, IDisposable
    {
        // Fields
        private static readonly PediatricSensorData instance = new PediatricSensorData();
        private DebugLog DebugLog = DebugLog.Instance;

        // Constructors
        static PediatricSensorData()
        {
        }

        private PediatricSensorData()
        {
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Subscribe(DataLayerEventHandler);
            if (!System.IO.Directory.Exists(SensorConfigFolderAbsolute))
                System.IO.Directory.CreateDirectory(SensorConfigFolderAbsolute);
        }

        // Properties
        public static PediatricSensorData Instance
        {
            get
            {
                return instance;
            }
        }

        public bool IsDisposed { get; private set; } = false;

        public bool DebugMode { get; set; } = false;
        public bool CanUpdateSeriesCollection { get; private set; } = true;
        public bool SaveDataEnabled { get; set; } = false;
        public bool SaveRAWValues { get; set; } = false;
        public string SaveFolder { get; set; } = String.Empty;
        public string SaveFolderCurrentRun { get; set; } = String.Empty;
        public string SaveSuffix { get; set; } = String.Empty;
        public string SensorConfigFolderAbsolute { get; private set; } =
            System.IO.Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                PediatricSoftConstants.PediatricSoftFolderRelative,
                PediatricSoftConstants.SensorConfigFolderRelative);

        public ObservableCollection<PediatricSensor> Sensors { get; private set; } = new ObservableCollection<PediatricSensor>();

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
            get { return DebugMode || canSendCommands; }
            private set { canSendCommands = value; RaisePropertyChanged(); }
        }

        private bool canScan = true;
        public bool CanScan
        {
            get { return DebugMode || canScan; }
            private set { canScan = value; RaisePropertyChanged(); }
        }

        private bool canLock = false;
        public bool CanLock
        {
            get { return DebugMode || canLock; }
            set { canLock = value; RaisePropertyChanged(); }
        }

        private bool canStartStop = false;
        public bool CanStartStop
        {
            get { return DebugMode || canStartStop; }
            set { canStartStop = value; RaisePropertyChanged(); }
        }

        private bool canZeroFields = false;
        public bool CanZeroFields
        {
            get { return DebugMode || canZeroFields; }
            set { canZeroFields = value; RaisePropertyChanged(); }
        }

        public bool IsMasterCardPresent { get; private set; } = false;

        private PediatricSoftConstants.DataSelect dataSelect = PediatricSoftConstants.DataSelect.ClosedLoop;
        public PediatricSoftConstants.DataSelect DataSelect
        {
            get { return dataSelect; }
            set { dataSelect = value; RaisePropertyChanged(); }
        }

        private PediatricSoftConstants.MagnetometerMode magnetometerMode = PediatricSoftConstants.MagnetometerMode.ClosedLoop;
        public PediatricSoftConstants.MagnetometerMode MagnetometerMode
        {
            get { return magnetometerMode; }
            set { magnetometerMode = value; RaisePropertyChanged(); }
        }

        // Methods
        public static PediatricSensorData GetInstance() { return instance; }

        public void ScanPortsAsync()
        {
            Task.Run(() =>
            {
                if (CanScan)
                {
                    DebugLog.Enqueue("Begin port scan");

                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    // Remove all failed sensors
                    for (int i = Sensors.Count - 1; i >= 0; i--)
                    {
                        if (Sensors[i].State == PediatricSoftConstants.SensorState.Failed)
                        {
                            Sensors[i].Dispose();
                            while (!Sensors[i].IsDisposed)
                            {
                                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                            }
                            App.Current.Dispatcher.Invoke(() => Sensors.RemoveAt(i));
                        }
                    }

                    RaisePropertyChanged("SensorCount");

                    string[] potentialSensorSerialNumbers = GetPotentialSensorSerialNumbers();
                    string[] currentSensorSerialNumbers = Sensors.Select(x => x.SN).ToArray();

                    // Try to add all potential sensors that are not on the list currently
                    foreach (string serial in potentialSensorSerialNumbers)
                    {
                        if (!currentSensorSerialNumbers.Contains(serial))
                        {
                            PediatricSensor sensor = new PediatricSensor(serial);
                            if (sensor.IsValid)
                            {
                                sensor.KickOffTasks();
                                App.Current.Dispatcher.Invoke(() => Sensors.Add(sensor));
                            }
                            else
                            {
                                while (!sensor.IsDisposed)
                                {
                                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                                }
                            }
                            RaisePropertyChanged("SensorCount");
                        }
                        else
                        {
                            DebugLog.Enqueue($"Sensor with serial number {serial} is already on the list - skipping");
                        }
                    }

                    if (Sensors.Count > 0)
                    {
                        CanLock = true;
                        CanSendCommands = true;
                    }

                    IsMasterCardPresent = false;
                    foreach (PediatricSensor sensor in Sensors)
                    {
                        IsMasterCardPresent = IsMasterCardPresent | sensor.IsMasterCard;
                    }

                    CanScan = true;

                    DebugLog.Enqueue("Port scan done");
                }
            });
        }

        public void StandbyAllAsync()
        {
            Task.Run(() =>
            {
                if (CanLock)
                {
                    DebugLog.Enqueue("Begin sensor standby");

                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    Parallel.ForEach(Sensors, sensor =>
                    {
                        if (sensor.State != PediatricSoftConstants.SensorState.Failed)
                        {
                            sensor.Standby();
                        }
                    });

                    CanScan = true;
                    CanLock = true;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = true;

                    DebugLog.Enqueue("Sensor standby done");
                }
            });
        }

        public void LockAllAsync()
        {
            Task.Run(() =>
            {
                if (CanLock)
                {
                    DebugLog.Enqueue("Begin sensor lock");

                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    Parallel.ForEach(Sensors, sensor =>
                    {
                        if (sensor.State != PediatricSoftConstants.SensorState.Failed)
                        {
                            sensor.Lock();
                        }
                    });

                    CanScan = true;
                    CanLock = true;
                    CanStartStop = true;
                    CanZeroFields = true;
                    CanSendCommands = true;

                    DebugLog.Enqueue("Sensor lock done");
                }
            });
        }

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
                        DebugLog.Enqueue("Stopping all sensors");

                        Parallel.ForEach(Sensors, sensor =>
                        {
                            if (sensor.State != PediatricSoftConstants.SensorState.Failed)
                            {
                                sensor.Stop();
                            }
                        });

                        IsRunning = false;

                        CanScan = true;
                        CanLock = true;
                        CanZeroFields = true;
                        CanSendCommands = true;
                    }
                    else
                    {
                        DebugLog.Enqueue("Starting all sensors");

                        IsRunning = true;

                        if (SaveDataEnabled) CreateDataFolder();

                        Parallel.ForEach(Sensors, sensor =>
                        {
                            if (sensor.State != PediatricSoftConstants.SensorState.Failed)
                            {
                                sensor.Start();
                            }
                        });
                    }

                    CanStartStop = true;
                }
            });
        }

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
                        if (sensor.State != PediatricSoftConstants.SensorState.Failed)
                        {
                            sensor.ZeroFields();
                        }
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
            if (CanUpdateSeriesCollection)
            {
                int count = 0;

                foreach (PediatricSensor sensor in Sensors)
                {
                    if (sensor.IsPlotted)
                    {
                        count++;
                    }
                }

                if (count > 0)
                {
                    PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Publish("ShowPlotWindow");
                    PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Publish("UpdateSeriesCollection");
                }
                else
                {
                    PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Publish("ClosePlotWindow");
                }
            }
        }

        public void ClearAll()
        {
            Parallel.ForEach(Sensors, sensor =>
            {
                sensor.Dispose();
                while (!sensor.IsDisposed)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                }
            });
            Sensors.Clear();
            RaisePropertyChanged("SensorCount");
        }

        public void ClearFFTAll()
        {
            Parallel.ForEach(Sensors, sensor =>
            {
                if (sensor.IsPlotted)
                {
                    sensor.InitializeDataFFTSingleSided();
                }
            });
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
                DebugLog.Enqueue("Number of FTDI devices (A and B): " + ftdiDeviceCount.ToString());
            }
            else
            {
                DebugLog.Enqueue("Failed to get number of devices (error " + ftStatus.ToString() + ")");
                return SerialNumbers.ToArray();
            }

            // If no devices available, return
            if (ftdiDeviceCount == 0)
            {
                DebugLog.Enqueue("No FTDI devices found");
                return SerialNumbers.ToArray();
            }

            // Allocate storage for device info list
            FTDI.FT_DEVICE_INFO_NODE[] ftdiDeviceList = new FTDI.FT_DEVICE_INFO_NODE[ftdiDeviceCount];

            // Wrap the following code in try-catch to avoid exceptions when devices are being added
            try
            {
                // Populate our device list
                ftStatus = myFtdiDevice.GetDeviceList(ftdiDeviceList);

                // Add potential sensors to the list
                if (ftStatus == FTDI.FT_STATUS.FT_OK)
                {
                    for (UInt32 i = 0; i < ftdiDeviceCount; i++)
                    {
                        if (ftdiDeviceList[i].Description.ToString() == PediatricSoftConstants.ValidIDN)
                        {
                            SerialNumbers.Add(ftdiDeviceList[i].SerialNumber.ToString());
                        }
                    }
                }
                else
                {
                    DebugLog.Enqueue("Failed to get FTDI serial numbers (error " + ftStatus.ToString() + ")");
                    return SerialNumbers.ToArray();
                }
            }
            catch
            {
                DebugLog.Enqueue("Failed to get FTDI serial numbers - devices are still being added. Please wait a few minutes.");
                return SerialNumbers.ToArray();
            }

            return SerialNumbers.ToArray();
        }

        private void ClearAllPlotCheckBox()
        {
            CanUpdateSeriesCollection = false;
            Parallel.ForEach(Sensors, sensor =>
            {
                if (sensor.IsPlotted)
                {
                    sensor.IsPlotted = false;
                }
            });
            CanUpdateSeriesCollection = true;
        }

        private void DataLayerEventHandler(string eventString)
        {
            switch (eventString)
            {
                case "Shutdown":
                    Dispose();
                    break;

                case "ClearAllPlotCheckBox":
                    ClearAllPlotCheckBox();
                    break;

                default:
                    break;
            }
        }

        private void CreateDataFolder()
        {
            if (String.IsNullOrEmpty(SaveSuffix))
            {
                SaveFolderCurrentRun = System.IO.Path.Combine(
                    SaveFolder,
                    DateTime.Now.ToString("yyyy-MM-dd_HHmmss"));
            }
            else
            {
                SaveFolderCurrentRun = System.IO.Path.Combine(
                    SaveFolder,
                    String.Concat(DateTime.Now.ToString("yyyy-MM-dd_HHmmss"), "_", Regex.Replace(SaveSuffix, @"[^\w]", "")));
            }
            System.IO.Directory.CreateDirectory(SaveFolderCurrentRun);
        }

        public void Dispose()
        {
            ClearAll();
            IsDisposed = true;
        }

    }
}