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

namespace PediatricSoft
{
    public class PediatricSensor : INotifyPropertyChanged
    {

        private SerialPort _SerialPort = new SerialPort()
        {
            WriteTimeout = PediatricSensorData.DefaultSerialPortWriteTimeout,
            ReadTimeout = PediatricSensorData.DefaultSerialPortReadTimeout,
            BaudRate = PediatricSensorData.DefaultSerialPortBaudRate,
            DtrEnable = false,
            RtsEnable = false
        };

        private ConcurrentQueue<DataPoint> data = new ConcurrentQueue<DataPoint>();
        private bool shouldBeRunning = false;
        private Task processingTask;
        private string filePath = String.Empty;
        private StreamWriter file;

        public bool ShouldBePlotted { get; set; } = false;
        public GearedValues<ObservableValue> _ChartValues = new GearedValues<ObservableValue>().WithQuality(Quality.Low);
        public double LastValue { get; private set; } = 0;

        public string Port { get; }
        public string IDN { get; }
        public string SN { get; }
        public bool IsRunning { get; private set; } = false;


        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged(string prop)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop));
        }

        private PediatricSensor() { }
        public PediatricSensor(SensorScanItem _SensorScanItem)
        {
            _SerialPort.PortName = _SensorScanItem.port;
            Port = _SensorScanItem.port;
            IDN = _SensorScanItem.idn;
            SN = _SensorScanItem.sn;
        }

        public void PediatricSensorStart()
        {
            IsRunning = true;
            filePath = System.IO.Path.Combine(PediatricSensorData.dataFolder, SN);
            filePath += ".txt";
            if (PediatricSensorData.SaveDataEnabled) file = File.AppendText(filePath);
            if (!_SerialPort.IsOpen) _SerialPort.Open();
            _SerialPort.DiscardOutBuffer();
            Thread.Sleep(PediatricSensorData.DefaultSerialPortSleepTime);
            _SerialPort.DiscardInBuffer();
            _SerialPort.WriteLine(PediatricSensorData.CommandStringStart);
            shouldBeRunning = true;
            processingTask = Task.Run(() => PediatricSensorProcessData());
            if (PediatricSensorData.IsDebugEnabled) Console.WriteLine($"Started sensor {SN}");
        }

        private void PediatricSensorProcessData()
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
                catch (TimeoutException)
                {
                    dataReadString = String.Empty;
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
                    LastValue = dataY;
                    OnPropertyChanged("LastValue");

                    if (PediatricSensorData.SaveDataEnabled) file.WriteLine(dataWriteString);
                }
                


            }


        }

        public void PediatricSensorStop()
        {
            shouldBeRunning = false;
            if (processingTask != null)
            {
                while (!processingTask.IsCompleted)
                {
                    if (PediatricSensorData.IsDebugEnabled) Console.WriteLine($"Waiting for sensor {SN} to stop");
                };
                processingTask.Dispose();
            };
            if (_SerialPort.IsOpen)
            {
                _SerialPort.DiscardOutBuffer();
                _SerialPort.WriteLine(PediatricSensorData.CommandStringStop);
                Thread.Sleep(PediatricSensorData.DefaultSerialPortSleepTime);
                _SerialPort.DiscardInBuffer();
                Thread.Sleep(PediatricSensorData.DefaultSerialPortSleepTime);
                _SerialPort.DiscardInBuffer();
                _SerialPort.Close();
            }
            _SerialPort.Dispose();
            if (PediatricSensorData.SaveDataEnabled && (file != null)) file.Dispose();
            IsRunning = false;
            if (PediatricSensorData.IsDebugEnabled) Console.WriteLine($"Sensor {SN} is stopped");
        }

        ~PediatricSensor()
        {
            PediatricSensorStop();
        }

    }
}