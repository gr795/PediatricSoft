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
        public static readonly bool IsDebugEnabled = true;
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

        private ObservableCollection<PediatricSensor> _PediatricSensorList = new ObservableCollection<PediatricSensor>();
        private bool pediatricSensorsRunning = false;

        public MainWindow()
        {
            InitializeComponent();
            sensorListView.ItemsSource = _PediatricSensorList;
        }

        private async void ButtonScanPorts_Click(object sender, RoutedEventArgs e)
        {
            buttonRunSensors.IsEnabled = false;
            if (!PediatricSensorScan.IsScanning)
            {
                logTextBox.AppendText("Clearing sensor list\n");
                foreach (PediatricSensor _PediatricSensor in _PediatricSensorList)
                {
                    _PediatricSensor.PediatricSensorStop();
                }
                _PediatricSensorList.Clear();
                logTextBox.AppendText("Scanning COM ports...\n");
                await PediatricSensorScan.StartScanAsync();
                foreach (string _string in PediatricSensorScan.DebugLogList.ToArray())
                {
                    logTextBox.AppendText(_string + "\n");
                }
                logTextBox.AppendText($"Found {PediatricSensorScan.SensorList.Count} sensors\n");
                logTextBox.ScrollToEnd();
                foreach (SensorScanItem _SensorScanItem in PediatricSensorScan.SensorList)
                {
                    _PediatricSensorList.Add(new PediatricSensor(_SensorScanItem));
                }
                logTextBox.AppendText($"Current sensor list is {_PediatricSensorList.Count} sensors\n");
                logTextBox.ScrollToEnd();
                if (_PediatricSensorList.Count > 0) buttonRunSensors.IsEnabled = true;
            }
        }

        private void ButtonRunSensors_Click(object sender, RoutedEventArgs e)
        {
            if (!pediatricSensorsRunning)
            {
                buttonRunSensors.IsEnabled = false;
                buttonScanPorts.IsEnabled = false;
                pediatricSensorsRunning = true;
                foreach (PediatricSensor _PediatricSensor in _PediatricSensorList)
                {

                    _PediatricSensor.PediatricSensorStart();
                }
                buttonRunSensors.IsEnabled = true;
            }
            else
            {
                buttonRunSensors.IsEnabled = false;
                foreach (PediatricSensor _PediatricSensor in _PediatricSensorList)
                {
                    _PediatricSensor.PediatricSensorStop();
                }
                pediatricSensorsRunning = false;
                buttonScanPorts.IsEnabled = true;
                buttonRunSensors.IsEnabled = true;
            }
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            foreach (PediatricSensor _PediatricSensor in _PediatricSensorList)
            {
                _PediatricSensor.PediatricSensorStop();
            }
        }

    }

    static class PediatricSensorData
    {

    }

    static class PediatricSensorScan
    {

        public static List<string> DebugLogList { get; } = new List<string>();
        public static List<SensorScanItem> SensorList { get; } = new List<SensorScanItem>();
        public static bool IsScanning { get; private set; } = false;

        public static async Task StartScanAsync()
        {
            if (!IsScanning)
            {
                IsScanning = true;
                DebugLogList.Clear();
                SensorList.Clear();
                await Task.Run(() => ScanForSensors());
                IsScanning = false;
            }
        }

        private static void ScanForSensors()
        {
            DebugLogList.Add("The following serial ports were found:");

            // Display each port name to the console.
            foreach (string port in SerialPort.GetPortNames())
            {
                SensorScanItem _SensorScanItem = ProbeComPort(port);
                if (_SensorScanItem.isValid)
                {
                    DebugLogList.Add(String.Concat(_SensorScanItem.port, " -> Found sensor ",
                        _SensorScanItem.idn, " with S/N ", _SensorScanItem.sn));
                    SensorList.Add(_SensorScanItem);
                }
                else
                    DebugLogList.Add(port);
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