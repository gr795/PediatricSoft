using System;
using System.Windows;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Concurrent;
using System.Text.RegularExpressions;
using System.ComponentModel;

namespace PediatricSoft
{

    public static class PediatricSoftConstants
    {
        public static readonly int DefaultSerialPortBaudRate = 115200;
        public static readonly int DefaultSerialPortWriteTimeout = 100;
        public static readonly int DefaultSerialPortReadTimeout = 100;
        public static readonly int DefaultSerialPortSleepTime = 100;
        public static readonly int DefaultSerialPortShutDownLoopDelay = 1;
        public static readonly string ValidIDN = "PediatricSensor";

        public static readonly int MaxQueueLength = 1000;
        public static bool IsDebugEnabled = true;
    }

    struct SensorScanItem
    {
        public bool isValid;
        public string port;
        public string idn;
        public string sn;
    }

    struct DataPoint
    {
        public double x;
        public double y;

        public DataPoint(double _x, double _y)
        {
            x = _x;
            y = _y;
        }
    }

    public partial class MainWindow : Window
    {

        public MainWindow()
        {
            InitializeComponent();
            sensorListView.ItemsSource = PediatricSensorData.Data;
        }

        private async void ButtonScanPorts_Click(object sender, RoutedEventArgs e)
        {
            buttonScanPorts.IsEnabled = false;
            buttonRunSensors.IsEnabled = false;
            if (!PediatricSensorScan.IsScanning)
            {
                if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine("Clearing sensor list\n");
                PediatricSensorData.ClearAll();

                if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine("Scanning COM ports...\n");
                await PediatricSensorScan.StartScanAsync();
                if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine($"Found {PediatricSensorScan.SensorScanList.Count} sensors\n");

                PediatricSensorData.AddAll();
                if (PediatricSensorData.Data.Count > 0) buttonRunSensors.IsEnabled = true;
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
        }

    }

    static class PediatricSensorData
    {
        private static ObservableCollection<PediatricSensor> data = new ObservableCollection<PediatricSensor>();
        private static bool pediatricSensorsRunning = false;

        public static ObservableCollection<PediatricSensor> Data { get { return data; } }
        public static bool IsRunning { get { return pediatricSensorsRunning; } }

        public static void AddAll()
        {
            ClearAll();
            foreach (SensorScanItem _SensorScanItem in PediatricSensorScan.SensorScanList)
            {
                data.Add(new PediatricSensor(_SensorScanItem));
            }
        }

        public static void StartAll()
        {
            if (!pediatricSensorsRunning)
            {
                pediatricSensorsRunning = true;
                foreach (PediatricSensor _PediatricSensor in data)
                {

                    _PediatricSensor.PediatricSensorStart();
                }
            }
        }

        public static void StopAll()
        {
            if (pediatricSensorsRunning)
            {
                foreach (PediatricSensor _PediatricSensor in data)
                {
                    _PediatricSensor.PediatricSensorStop();
                }
                pediatricSensorsRunning = false;
            }
        }

        public static void ClearAll()
        {
            StopAll();
            data.Clear();
        }

    }

    static class PediatricSensorScan
    {

        public static List<SensorScanItem> SensorScanList { get; } = new List<SensorScanItem>();
        public static bool IsScanning { get; private set; } = false;

        public static async Task StartScanAsync()
        {
            if (!IsScanning)
            {
                IsScanning = true;
                SensorScanList.Clear();
                await Task.Run(() => ScanForSensors());
                IsScanning = false;
            }
        }

        private static void ScanForSensors()
        {
            if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine("The following serial ports were found:");

            // Display each port name to the console.
            foreach (string port in SerialPort.GetPortNames())
            {
                SensorScanItem _SensorScanItem = ProbeComPort(port);
                if (_SensorScanItem.isValid)
                {
                    if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine
                            (String.Concat(_SensorScanItem.port, " -> Found sensor ",
                        _SensorScanItem.idn, " with S/N ", _SensorScanItem.sn));
                    SensorScanList.Add(_SensorScanItem);
                }
                else
                    if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine(port);
            }
        }

        private static SensorScanItem ProbeComPort(string port)
        {
            SensorScanItem _SensorScanItem;
            _SensorScanItem.isValid = false;
            _SensorScanItem.port = port;
            _SensorScanItem.idn = "";
            _SensorScanItem.sn = "";

            SerialPort _SerialPort = new SerialPort()
            {
                PortName = port,
                WriteTimeout = PediatricSoftConstants.DefaultSerialPortWriteTimeout,
                ReadTimeout = PediatricSoftConstants.DefaultSerialPortReadTimeout,
                BaudRate = PediatricSoftConstants.DefaultSerialPortBaudRate
            };

            if (!_SerialPort.IsOpen) _SerialPort.Open();
            _SerialPort.DiscardOutBuffer();
            _SerialPort.WriteLine("*STOP?");
            Thread.Sleep(PediatricSoftConstants.DefaultSerialPortSleepTime);
            _SerialPort.DiscardInBuffer();

            try
            {
                _SerialPort.WriteLine("*IDN?");
                _SensorScanItem.idn = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                _SerialPort.WriteLine("*SN?");
                _SensorScanItem.sn = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
            }
            catch (TimeoutException) { }

            if (_SerialPort.IsOpen) _SerialPort.Close();
            _SerialPort.Dispose();

            if (_SensorScanItem.idn.Contains(PediatricSoftConstants.ValidIDN)) _SensorScanItem.isValid = true;
            return _SensorScanItem;
        }
    }

    class PediatricSensor : INotifyPropertyChanged
    {

        private SerialPort _SerialPort = new SerialPort()
        {
            WriteTimeout = PediatricSoftConstants.DefaultSerialPortWriteTimeout,
            ReadTimeout = PediatricSoftConstants.DefaultSerialPortReadTimeout,
            BaudRate = PediatricSoftConstants.DefaultSerialPortBaudRate
        };

        private ConcurrentQueue<DataPoint> data = new ConcurrentQueue<DataPoint>();
        private bool shouldBeRunning = false;
        private Task processingTask;

        public string Port { get; }
        public string IDN { get; }
        public string SN { get; }
        public bool IsRunning { get; private set; }

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged(string prop)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop));
        }

        public double DataPoint0y { get; private set; }

        public PediatricSensor(SensorScanItem _SensorScanItem)
        {
            _SerialPort.PortName = _SensorScanItem.port;
            Port = _SensorScanItem.port;
            IDN = _SensorScanItem.idn;
            SN = _SensorScanItem.sn;
            data.Enqueue(new DataPoint());
        }

        public void PediatricSensorStart()
        {
            IsRunning = true;
            if (!_SerialPort.IsOpen) _SerialPort.Open();
            _SerialPort.DiscardOutBuffer();
            Thread.Sleep(PediatricSoftConstants.DefaultSerialPortSleepTime);
            _SerialPort.DiscardInBuffer();
            _SerialPort.WriteLine("*START?");
            shouldBeRunning = true;
            processingTask = Task.Run(() => PediatricSensorProcessData());
        }
        private void PediatricSensorProcessData()
        {
            DataPoint dummyDataPoint = new DataPoint();
            string dataReadString;
            string dataXString;
            string dataYString;
            while (shouldBeRunning)
            {
                dataReadString = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                dataXString = dataReadString.Split('@')[0];
                dataYString = dataReadString.Split('@')[1];
                data.Enqueue(new DataPoint(Convert.ToDouble(dataXString), Convert.ToDouble(dataYString)));
                if (data.Count > PediatricSoftConstants.MaxQueueLength) while (!data.TryDequeue(out dummyDataPoint)) { };
                DataPoint0y = Convert.ToDouble(dataYString);
                OnPropertyChanged("DataPoint0y");
            }
        }

        public void PediatricSensorStop()
        {
            shouldBeRunning = false;
            if (processingTask != null)
            {
                while (!processingTask.IsCompleted)
                {
                    if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine($"Waiting for sensor {SN} to stop");
                };
                processingTask.Dispose();
            };
            if (_SerialPort.IsOpen)
            {
                _SerialPort.DiscardOutBuffer();
                _SerialPort.WriteLine("*STOP?");
                Thread.Sleep(PediatricSoftConstants.DefaultSerialPortSleepTime);
                _SerialPort.DiscardInBuffer();
                Thread.Sleep(PediatricSoftConstants.DefaultSerialPortSleepTime);
                _SerialPort.DiscardInBuffer();
                _SerialPort.Close();
            }
            _SerialPort.Dispose();
            IsRunning = false;
            if (PediatricSoftConstants.IsDebugEnabled) Console.WriteLine($"Sensor {SN} is stopped");
        }

        ~PediatricSensor()
        {
            PediatricSensorStop();
        }

    }
}