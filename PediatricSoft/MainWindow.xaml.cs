using System;
using System.Windows;
using System.IO.Ports;
using System.Threading;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Text.RegularExpressions;
using System.Collections.ObjectModel;

namespace PediatricSoft
{

    public static class PediatricSensorDefaults
    {
        public static int DefaultSerialPortBaudRate { get; } = 115200;
        public static int DefaultSerialPortWriteTimeout { get; } = 100;
        public static int DefaultSerialPortReadTimeout { get; } = 100;
        public static int DefaultSerialPortSleepTime { get; } = 100;
        public static string ValidIDN { get; } = "PediatricSensor";
    }

    struct SensorScanItem
    {
        public string port;
        public string idn;
        public string sn;
    }

    public partial class MainWindow : Window
    {
        private PediatricSensorScan _PediatricSensorScan = new PediatricSensorScan();
        private ObservableCollection<PediatricSensor> _PediatricSensorList = new ObservableCollection<PediatricSensor>();

        public MainWindow()
        {
            InitializeComponent();
            sensorListView.ItemsSource = _PediatricSensorList;
        }

        private async void ButtonScanPorts_Click(object sender, RoutedEventArgs e)
        {
            if (!_PediatricSensorScan.Scanning)
            {
                logTextBox.AppendText("Clearing sensor list\n");
                _PediatricSensorList.Clear();
                logTextBox.AppendText("Scanning COM ports...\n");
                await _PediatricSensorScan.StartScanAsync();
                foreach (string _string in _PediatricSensorScan.DebugLogList.ToArray())
                {
                    logTextBox.AppendText(_string + "\n");
                }
                logTextBox.AppendText($"Found {_PediatricSensorScan.SensorList.Count} sensors\n");
                logTextBox.ScrollToEnd();
                foreach (SensorScanItem _SensorScanItem in _PediatricSensorScan.SensorList)
                {
                    _PediatricSensorList.Add(new PediatricSensor(_SensorScanItem));
                }
                logTextBox.AppendText($"Current sensor list is {_PediatricSensorList.Count} sensors\n");
                logTextBox.ScrollToEnd();
            }
            //sensorListView.UpdateLayout();
            //UpdateSensorList();
        }

        private void UpdateSensorList()
        {
            int i = 0;
            //sensorListView.ClearValue;
            foreach (PediatricSensor _PediatricSensor in _PediatricSensorList)
            {
                logTextBox.AppendText($"Updating sensor {i}\n");
                logTextBox.AppendText($"{_PediatricSensor.SN}\n");
                logTextBox.ScrollToEnd();
                i++;

            }
        }

    }

    class PediatricSensorScan
    {

        public List<string> DebugLogList { get; } = new List<string>();
        public List<SensorScanItem> SensorList { get; } = new List<SensorScanItem>();
        public bool Scanning { get; set; } = false;

        public async Task StartScanAsync()
        {
            if (!Scanning)
            {
                Scanning = true;
                DebugLogList.Clear();
                SensorList.Clear();
                await Task.Run(() => ScanForSensors());
                Scanning = false;
            }
        }

        private void ScanForSensors()
        {
            DebugLogList.Add("The following serial ports were found:");
            // Get a list of serial port names.
            string[] portNames = SerialPort.GetPortNames();

            // Display each port name to the console.
            foreach (string port in portNames)
            {
                SensorScanItem _SensorScanItem = ProbeComPort(port);
                if (_SensorScanItem.idn.Contains(PediatricSensorDefaults.ValidIDN))
                {
                    DebugLogList.Add(String.Concat("Probing port ", _SensorScanItem.port, " -> ",
                        "Found sensor ", _SensorScanItem.idn, " with S/N ", _SensorScanItem.sn));
                    SensorList.Add(_SensorScanItem);
                }
                else
                    DebugLogList.Add(String.Concat("Probing port ", port));
            }
        }

        private SensorScanItem ProbeComPort(string port)
        {
            SensorScanItem _SensorScanItem;
            _SensorScanItem.port = port;
            _SensorScanItem.idn = "";
            _SensorScanItem.sn = "";

            // Create a new SerialPort object with default settings.
            SerialPort _SerialPort = new SerialPort(port, PediatricSensorDefaults.DefaultSerialPortBaudRate)
            {
                WriteTimeout = PediatricSensorDefaults.DefaultSerialPortWriteTimeout,
                ReadTimeout = PediatricSensorDefaults.DefaultSerialPortReadTimeout
            };
            try
            {
                _SerialPort.Open();
                _SerialPort.WriteLine("*STOP?");
                Thread.Sleep(PediatricSensorDefaults.DefaultSerialPortSleepTime);
                _SerialPort.DiscardInBuffer();
                _SerialPort.WriteLine("*IDN?");
                _SensorScanItem.idn = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                _SerialPort.WriteLine("*SN?");
                _SensorScanItem.sn = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                _SerialPort.Close();
            }
            catch (TimeoutException) { }
            _SerialPort.Close();
            return _SensorScanItem;
        }
    }

    class PediatricSensor
    {
        public string Port { get; } = "";
        public string IDN { get; } = "";
        public string SN { get; } = "";
        public bool IsRunning { get; } = false;

        private SerialPort _SerialPort = new SerialPort()
        {
            WriteTimeout = PediatricSensorDefaults.DefaultSerialPortWriteTimeout,
            ReadTimeout = PediatricSensorDefaults.DefaultSerialPortReadTimeout,
            BaudRate = PediatricSensorDefaults.DefaultSerialPortBaudRate
        };

        public PediatricSensor(SensorScanItem _SensorScanItem)
        {
            _SerialPort.PortName = _SensorScanItem.port;
            Port = _SensorScanItem.port;
            IDN = _SensorScanItem.idn;
            SN = _SensorScanItem.sn;
        }

        public void PediatricSensorStart()
        {
            
            try
            {
                _SerialPort.Open();
                _SerialPort.WriteLine("*START?");
            }
            catch (TimeoutException) { }
        }

        public void PediatricSensorStop()
        {
            try
            {
                _SerialPort.WriteLine("*STOP?");
            }
            catch (TimeoutException) { }
            _SerialPort.Close();
        }

        ~PediatricSensor()
        {
            //PediatricSensorStop();
        }

    }
}