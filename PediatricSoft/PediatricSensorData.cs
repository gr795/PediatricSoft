using FTD2XX_NET;
using Prism.Mvvm;
using System;
using System.Linq;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace PediatricSoft
{
    public sealed class PediatricSensorData : BindableBase, IDisposable
    {
        // Fields
        private static readonly PediatricSensorData instance = new PediatricSensorData();

        // Constructors
        static PediatricSensorData()
        {
        }

        private PediatricSensorData()
        {
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Subscribe(DataLayerEventHandler);
            if (System.IO.Directory.Exists(SensorConfigFolderAbsolute))
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Sensor config directory exists");
            else
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Sensor config directory doesn't exist - creating");
                System.IO.Directory.CreateDirectory(SensorConfigFolderAbsolute);
            }
        }

        // Properties
        public static PediatricSensorData Instance
        {
            get
            {
                return instance;
            }
        }

        public bool DebugMode { get; set; } = false;
        public bool SaveDataEnabled { get; set; } = false;
        public bool SaveRAWValues { get; set; } = false;
        public string SaveFolder { get; set; } = String.Empty;
        public string SaveFolderCurrentRun { get; set; } = String.Empty;
        public string SaveSuffix { get; set; } = String.Empty;
        public string SensorConfigFolderAbsolute { get; private set; } = System.IO.Path.Combine(System.IO.Directory.GetCurrentDirectory(), PediatricSoftConstants.SensorConfigFolderRelative);

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

        private PediatricSoftConstants.DataSelect dataSelect = PediatricSoftConstants.DataSelect.ADC;
        public PediatricSoftConstants.DataSelect DataSelect
        {
            get { return dataSelect; }
            set { dataSelect = value; }
        }

        // Methods
        public static PediatricSensorData GetInstance() { return instance; }

        public void ScanPortsAsync()
        {
            Task.Run(() =>
            {
                if (CanScan)
                {
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Begin port scan");

                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    // Remove all failed sensors
                    Parallel.ForEach(Sensors, sensor =>
                        {
                            if (sensor.State == PediatricSoftConstants.SensorState.Failed)
                            {
                                sensor.Dispose();
                                while (!sensor.IsDisposed)
                                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                                App.Current.Dispatcher.Invoke(() => Sensors.Remove(sensor));
                            }
                        });

                    string[] potentialSensorSerialNumbers = GetPotentialSensorSerialNumbers();
                    string[] currentSensorSerialNumbers = Sensors.Select(x => x.SN).ToArray();

                    // Try to add all potential sensors that are not on the list currently
                    Parallel.ForEach(potentialSensorSerialNumbers, serial =>
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
                                while (!sensor.IsDisposed)
                                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                            RaisePropertyChanged("SensorCount");
                        }
                        else
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor with serial number {serial} is already on the list - skipping");
                    });

                    if (Sensors.Count > 0)
                    {
                        CanLock = true;
                        CanSendCommands = true;
                    }

                    CanScan = true;

                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Port scan done");
                }
            });
        }

        public void LockAllAsync()
        {
            Task.Run(() =>
            {
                if (CanLock)
                {
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Begin sensor lock");

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
                    CanStartStop = false;
                    CanZeroFields = true;
                    CanSendCommands = true;

                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Sensor lock done");
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
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Stopping all sensors");

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
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Starting all sensors");

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
                PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Publish("ClosePlotWindow");
        }

        public void ClearAll()
        {
            Parallel.ForEach(Sensors, _PediatricSensor =>
            {
                _PediatricSensor.Dispose();
                while (!_PediatricSensor.IsDisposed) Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
            });
            Sensors.Clear();
            RaisePropertyChanged("SensorCount");
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
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Number of FTDI devices: " + ftdiDeviceCount.ToString());
            }
            else
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Failed to get number of devices (error " + ftStatus.ToString() + ")");
                return SerialNumbers.ToArray();
            }

            // If no devices available, return
            if (ftdiDeviceCount == 0)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "FTDI devices found");
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
                    if (ftdiDeviceList[i].Description.ToString() == PediatricSoftConstants.ValidIDN)
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Adding a potential sensor with S/N: " + ftdiDeviceList[i].SerialNumber.ToString());
                        SerialNumbers.Add(ftdiDeviceList[i].SerialNumber.ToString());
                    }
                }
            }

            return SerialNumbers.ToArray();
        }

        private void ClearAllPlotCheckBox()
        {

            Parallel.ForEach(Sensors, sensor =>
            {
                if (sensor.IsPlotted)
                    sensor.IsPlotted = false;
            });

        }

        private void DataLayerEventHandler(string eventString)
        {
            switch (eventString)
            {
                case "Shutdown":
                    ClearAll();
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
            ClearAll();
        }

    }
}