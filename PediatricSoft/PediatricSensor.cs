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
            WriteTimeout = PediatricSoftConstants.DefaultSerialPortWriteTimeout,
            ReadTimeout = PediatricSoftConstants.DefaultSerialPortReadTimeout,
            BaudRate = PediatricSoftConstants.DefaultSerialPortBaudRate
        };

        private ConcurrentQueue<DataPoint> data = new ConcurrentQueue<DataPoint>();
        private bool shouldBeRunning = false;
        private Task processingTask;
        private string filePath = String.Empty;

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
            filePath = System.IO.Path.Combine(PediatricSensorData.dataFolder, SN);
            filePath += ".txt";
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
            string dataWriteString;
            string dataXString;
            string dataYString;
            double dataX;
            double dataY;
            using (StreamWriter file = File.AppendText(filePath) )
            {
                while (shouldBeRunning)
                {
                    dataReadString = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                    dataXString = dataReadString.Split('@')[0];
                    dataYString = dataReadString.Split('@')[1];
                    dataWriteString = String.Concat(dataXString, "\t", dataYString);
                    dataX = Convert.ToDouble(dataXString);
                    dataY = Convert.ToDouble(dataYString);
                    data.Enqueue(new DataPoint(dataX, dataY));
                    if (data.Count > PediatricSoftConstants.MaxQueueLength)
                        while (!data.TryDequeue(out dummyDataPoint)) { };
                    if (ShouldBePlotted)
                    {
                        _ChartValues.Add(new ObservableValue(dataY));
                        if (_ChartValues.Count > PediatricSoftConstants.MaxQueueLength) _ChartValues.RemoveAt(0);
                    }
                    LastValue = dataY;
                    OnPropertyChanged("LastValue");

                    file.WriteLine(dataWriteString);

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
                    if (PediatricSoftGlobals.IsDebugEnabled) Console.WriteLine($"Waiting for sensor {SN} to stop");
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

            if (PediatricSoftGlobals.IsDebugEnabled) Console.WriteLine($"Sensor {SN} is stopped");
        }

        ~PediatricSensor()
        {
            PediatricSensorStop();
        }

    }
}