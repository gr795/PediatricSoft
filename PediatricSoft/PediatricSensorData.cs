using System;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Text.RegularExpressions;
using LiveCharts;
using LiveCharts.Wpf;

namespace PediatricSoft
{
    public static class PediatricSensorData
    {

        public static ObservableCollection<PediatricSensor> Sensors = new ObservableCollection<PediatricSensor>();
        public static bool IsRunning { get; private set; } = false;
        public static List<SensorScanItem> SensorScanList = new List<SensorScanItem>();
        public static bool IsScanning { get; private set; } = false;

        public static SeriesCollection _SeriesCollection { get; set; } = new SeriesCollection();

        public static string dataFolder = String.Empty;

        static PediatricSensorData()
        {

        }

        public static void AddAll()
        {
            ClearAll();
            foreach (SensorScanItem _SensorScanItem in PediatricSensorData.SensorScanList)
            {
                Sensors.Add(new PediatricSensor(_SensorScanItem));
            }
        }

        public static void StartAll()
        {
            if (!IsRunning)
            {
                IsRunning = true;
                if (PediatricSoftGlobals.SaveData) CreateDataFolder();
                foreach (PediatricSensor _PediatricSensor in Sensors)
                {
                    _PediatricSensor.PediatricSensorStart();
                }
            }
        }

        public static void StopAll()
        {
            if (IsRunning)
            {
                foreach (PediatricSensor _PediatricSensor in Sensors)
                {
                    _PediatricSensor.PediatricSensorStop();
                }
                IsRunning = false;
            }
        }

        public static void ClearAll()
        {
            StopAll();
            Sensors.Clear();
        }

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

        private static void CreateDataFolder()
        {
            dataFolder = System.IO.Path.Combine(
                System.IO.Directory.GetCurrentDirectory(),
                PediatricSoftConstants.DefaultFolder,
                DateTime.Now.ToString("yyyy-MM-dd_HHmmss") );
            System.IO.Directory.CreateDirectory(dataFolder);
        }

        private static void ScanForSensors()
        {
            if (PediatricSoftGlobals.IsDebugEnabled) Console.WriteLine("The following serial ports were found:");

            // Display each port name to the console.
            foreach (string port in SerialPort.GetPortNames())
            {
                SensorScanItem _SensorScanItem = ProbeComPort(port);
                if (_SensorScanItem.isValid)
                {
                    if (PediatricSoftGlobals.IsDebugEnabled) Console.WriteLine
                            (String.Concat(_SensorScanItem.port, " -> Found sensor ",
                        _SensorScanItem.idn, " with S/N ", _SensorScanItem.sn));
                    SensorScanList.Add(_SensorScanItem);
                }
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
            catch (TimeoutException)
            {
                if (PediatricSoftGlobals.IsDebugEnabled) Console.WriteLine($"Timeout on port {port}");
            }

            if (_SerialPort.IsOpen) _SerialPort.Close();
            _SerialPort.Dispose();

            if (_SensorScanItem.idn.Contains(PediatricSoftConstants.ValidIDN)) _SensorScanItem.isValid = true;
            return _SensorScanItem;
        }

    }
}