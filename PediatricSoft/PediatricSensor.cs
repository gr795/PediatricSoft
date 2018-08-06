using System;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using System.Text.RegularExpressions;
using System.ComponentModel;
using LiveCharts;
using LiveCharts.Configurations;
using LiveCharts.Wpf;
using LiveCharts.Defaults;
using LiveCharts.Geared;
using System.IO;
using System.Diagnostics;

namespace PediatricSoft
{
    public class PediatricSensor : INotifyPropertyChanged
    {

        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private SerialPort _SerialPort;
        private ConcurrentQueue<DataPoint> data = new ConcurrentQueue<DataPoint>();
        private bool shouldBeRunning = false;
        private Task processingTask;
        private string filePath = String.Empty;
        private StreamWriter file;
        private bool isPortOpen = false;
        private int guiCounter = 0;

        public bool ShouldBePlotted { get; set; } = false;
        public GearedValues<ObservableValue> _ChartValues = new GearedValues<ObservableValue>().WithQuality(Quality.Low);
        public double LastValue { get; private set; } = 0;

        public string Port { get; private set; } = String.Empty;
        public string IDN { get; private set; } = String.Empty;
        public string SN { get; private set; } = String.Empty;
        public bool IsRunning { get; private set; } = false;
        public bool IsValid { get; private set; } = false;

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged(string prop)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop));
        }

        private PediatricSensor() { }
        public PediatricSensor(string port)
        {
            Port = port;

            _SerialPort = new SerialPort()
            {
                PortName = port,
                WriteTimeout = PediatricSensorData.DefaultSerialPortWriteTimeout,
                ReadTimeout = PediatricSensorData.DefaultSerialPortReadTimeout,
                BaudRate = PediatricSensorData.DefaultSerialPortBaudRate,
                DtrEnable = false,
                RtsEnable = false
            };

        }

        private void PortOpen()
        {
            if (!_SerialPort.IsOpen)
            {
                try
                {
                    _SerialPort.Open();
                    _SerialPort.DiscardOutBuffer();
                    _SerialPort.WriteLine(PediatricSensorData.CommandStringStop);
                    Thread.Sleep(PediatricSensorData.DefaultSerialPortSleepTime);
                    _SerialPort.DiscardInBuffer();
                    isPortOpen = true;
                }
                catch (Exception e) { if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine(e.Message); }
            }
        }

        private void PortClose()
        {
            if (_SerialPort.IsOpen)
            {
                try
                {
                    _SerialPort.DiscardOutBuffer();
                    _SerialPort.WriteLine(PediatricSensorData.CommandStringStop);
                    Thread.Sleep(PediatricSensorData.DefaultSerialPortSleepTime);
                    _SerialPort.DiscardInBuffer();
                    Thread.Sleep(PediatricSensorData.DefaultSerialPortSleepTime);
                    _SerialPort.DiscardInBuffer();
                    _SerialPort.Close();
                    isPortOpen = false;
                }
                catch (Exception e) { if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine(e.Message); }
                
            }
        }

        public void Validate()
        {
            PortOpen();
            if (isPortOpen)
            {
                try
                {
                    _SerialPort.WriteLine(PediatricSensorData.CommandStringIDN);
                    IDN = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                    _SerialPort.WriteLine(PediatricSensorData.CommandStringSN);
                    SN = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                }
                catch (Exception e)
                {
                    if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine($"Timeout on port {Port}");
                    if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine(e.Message);
                }
                if (IDN.Contains(PediatricSensorData.ValidIDN)) IsValid = true;
                OnPropertyChanged("IDN");
                OnPropertyChanged("SN");
                PortClose();
            }
        }

        public void Start()
        {
            if (IsValid)
            {
                PortOpen();
                IsRunning = true;
                filePath = System.IO.Path.Combine(PediatricSensorData.dataFolder, SN);
                filePath += ".txt";
                if (PediatricSensorData.SaveDataEnabled) file = File.AppendText(filePath);

                _SerialPort.WriteLine(PediatricSensorData.CommandStringStart);
                shouldBeRunning = true;
                processingTask = Task.Run(() => ProcessData());
                if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine($"Started sensor {SN}");
            }
        }

        private void ProcessData()
        {
            DataPoint dummyDataPoint = new DataPoint();
            string dataReadString = String.Empty;
            string dataReadSanitizedString = String.Empty;
            string dataWriteString = String.Empty;
            //string dataXString;
            //string dataYString;
            double dataX = 0;
            double dataY = 0;

            while (shouldBeRunning)
            {


                try
                {
                    dataReadString = _SerialPort.ReadLine();
                }
                catch (Exception e)
                {
                    dataReadString = String.Empty;
                    if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine(e.Message);
                }

                if (dataReadString.Length > 0)
                {
                    dataReadSanitizedString = Regex.Replace(dataReadString, @"[^\d]", "", RegexOptions.None, TimeSpan.FromSeconds(1));
                }
                else
                {
                    dataReadSanitizedString = String.Empty;
                }
                
                if (dataReadSanitizedString.Length > 0)
                {
                    //dataXString = dataReadString.Split('@')[0];
                    //dataYString = dataReadString.Split('@')[1];
                    dataWriteString = String.Concat(dataReadSanitizedString, "\t", dataReadSanitizedString);
                    dataX = Convert.ToDouble(dataReadSanitizedString);
                    dataY = Convert.ToDouble(dataReadSanitizedString);
                    data.Enqueue(new DataPoint(dataX, dataY));
                    if (data.Count > PediatricSensorData.MaxQueueLength)
                        while (!data.TryDequeue(out dummyDataPoint)) { };
                    if (ShouldBePlotted)
                    {
                        _ChartValues.Add(new ObservableValue(dataY));
                        if (_ChartValues.Count > PediatricSensorData.MaxQueueLength) _ChartValues.RemoveAt(0);
                    }

                    guiCounter++;
                    if (guiCounter > PediatricSensorData.GUICounter)
                    {
                        LastValue = dataY;
                        OnPropertyChanged("LastValue");
                        guiCounter = 0;
                    }

                    if (PediatricSensorData.SaveDataEnabled) file.WriteLine(dataWriteString);

                    //Thread.Sleep(10);
                }
                


            }


        }

        public void Stop()
        {
            if (IsValid)
            {
                shouldBeRunning = false;
                if (processingTask != null)
                {
                    while (!processingTask.IsCompleted)
                    {
                        if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine($"Waiting for sensor {SN} to stop");
                    };
                    processingTask.Dispose();
                };
                PortClose();
                
                if (PediatricSensorData.SaveDataEnabled && (file != null)) file.Dispose();
                IsRunning = false;
                if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine($"Sensor {SN} is stopped");
            }
        }

        ~PediatricSensor()
        {
            Stop();
            _SerialPort.Dispose();
        }

    }
}