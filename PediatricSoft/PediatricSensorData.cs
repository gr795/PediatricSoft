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

            System.IO.Directory.CreateDirectory(SensorConfigFolderAbsolute);
        }

        public static PediatricSensorData Instance
        {
            get
            {
                return instance;
            }
        }

        public static PediatricSensorData GetInstance() { return instance; }



        // Globals
        public bool DebugMode { get; set; } = false;
        public bool SaveDataEnabled { get; set; } = false;
        public bool SaveRAWValues { get; set; } = false;
        public string SaveFolder { get; set; } = String.Empty;
        public string SaveFolderCurrentRun { get; set; } = String.Empty;
        public string SaveSuffix { get; set; } = String.Empty;
        public string CommandHistory { get; set; } = String.Empty;
        public string SensorConfigFolderAbsolute = System.IO.Path.Combine(System.IO.Directory.GetCurrentDirectory(), PediatricSoftConstants.SensorConfigFolderRelative);

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
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Begin port scan");

                    CanScan = false;
                    CanLock = false;
                    CanStartStop = false;
                    CanZeroFields = false;
                    CanSendCommands = false;

                    Parallel.ForEach(Sensors, sensor =>
                    {
                        if (sensor.State == PediatricSoftConstants.SensorState.Failed)
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
                                 Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                         RaisePropertyChanged("SensorCount");
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
                    CanStartStop = true;
                    CanZeroFields = true;
                    CanSendCommands = true;

                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Sensor lock done");
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