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
    public class PediatricSensorData
    {

        // Constants
        public static readonly int DefaultSerialPortBaudRate = 115200;
        public static readonly int DefaultSerialPortWriteTimeout = 500;
        public static readonly int DefaultSerialPortReadTimeout = 500;
        public static readonly int DefaultSerialPortSleepTime = 500;
        public static readonly int DefaultSerialPortShutDownLoopDelay = 1;

        public static readonly int NumberOfThreads = 32;
        public static readonly int MaxQueueLength = 1000;
        public static readonly string DefaultFolder = "Data";

        public static readonly string ValidIDN = "12";
        public static readonly string CommandStringIDN = "Q0";
        public static readonly string CommandStringSN = "Q1";
        public static readonly string CommandStringStart = "Q2";
        public static readonly string CommandStringStop = "Q3";

        // Globals
        public static bool IsDebugEnabled = true;
        public static bool IsPlotting = false;
        public static bool PlotWindowClosed = false;
        public static bool SaveDataEnabled { get; set; } = false;
        public static string SaveSuffix { get; set; } = String.Empty;

        public static ObservableCollection<PediatricSensor> Sensors { get; set; } = new ObservableCollection<PediatricSensor>();
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
                   if (PediatricSensorData.SaveDataEnabled) CreateDataFolder();
                   Parallel.ForEach(Sensors, _PediatricSensor =>
                   {
                       _PediatricSensor.PediatricSensorStart();
                   }
                   );
               }
        }

        public static void StopAll()
        {
                if (IsRunning)
                {
                    Parallel.ForEach(Sensors, _PediatricSensor =>
                    {
                        _PediatricSensor.PediatricSensorStop();
                    }
                    );
                    IsRunning = false;
                }
        }

        public static void ClearAll()
        {
            StopAll();
            Sensors.Clear();
        }

        public static void StartScan()
        {
            if (!IsScanning)
            {
                IsScanning = true;
                SensorScanList.Clear();
                ScanForSensors();
                IsScanning = false;
            }
        }

        public static void ValidatePrefixString()
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

        private static void CreateDataFolder()
        {
            if (String.IsNullOrEmpty(SaveSuffix))
                dataFolder = System.IO.Path.Combine(
                System.IO.Directory.GetCurrentDirectory(),
                PediatricSensorData.DefaultFolder,
                DateTime.Now.ToString("yyyy-MM-dd_HHmmss"));
            else
                dataFolder = System.IO.Path.Combine(
                    System.IO.Directory.GetCurrentDirectory(),
                    PediatricSensorData.DefaultFolder,
                    String.Concat(DateTime.Now.ToString("yyyy-MM-dd_HHmmss"), "_", SaveSuffix));
            System.IO.Directory.CreateDirectory(dataFolder);
        }

        private static void ScanForSensors()
        {
            if (PediatricSensorData.IsDebugEnabled) Console.WriteLine("The following serial ports were found:");

            Parallel.ForEach(SerialPort.GetPortNames(), port =>
            {
                SensorScanItem _SensorScanItem = ProbeComPort(port);
                if (_SensorScanItem.isValid)
                {
                    if (PediatricSensorData.IsDebugEnabled) Console.WriteLine
                            (String.Concat(_SensorScanItem.port, " -> Found sensor ",
                        _SensorScanItem.idn, " with S/N ", _SensorScanItem.sn));
                    SensorScanList.Add(_SensorScanItem);
                }
                else if (PediatricSensorData.IsDebugEnabled) Console.WriteLine(_SensorScanItem.port);
            });
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
                WriteTimeout = PediatricSensorData.DefaultSerialPortWriteTimeout,
                ReadTimeout = PediatricSensorData.DefaultSerialPortReadTimeout,
                BaudRate = PediatricSensorData.DefaultSerialPortBaudRate
            };

            if (!_SerialPort.IsOpen) _SerialPort.Open();
            _SerialPort.DiscardOutBuffer();
            _SerialPort.WriteLine(CommandStringStop);
            Thread.Sleep(PediatricSensorData.DefaultSerialPortSleepTime);
            _SerialPort.DiscardInBuffer();

            try
            {
                _SerialPort.WriteLine(CommandStringIDN);
                _SensorScanItem.idn = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                _SerialPort.WriteLine(CommandStringSN);
                _SensorScanItem.sn = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
            }
            catch (TimeoutException)
            {
                if (PediatricSensorData.IsDebugEnabled) Console.WriteLine($"Timeout on port {port}");
            }

            if (_SerialPort.IsOpen) _SerialPort.Close();
            _SerialPort.Dispose();

            if (_SensorScanItem.idn.Contains(PediatricSensorData.ValidIDN)) _SensorScanItem.isValid = true;
            return _SensorScanItem;
        }

    }
}