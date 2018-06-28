﻿using System;
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

        public bool Plot { get; set; } = true;
        public ChartValues<ObservableValue> _ChartValues = new ChartValues<ObservableValue>();
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
            double dataX;
            double dataY;
            while (shouldBeRunning)
            {
                dataReadString = Regex.Replace(_SerialPort.ReadLine(), @"\t|\n|\r", "");
                dataXString = dataReadString.Split('@')[0];
                dataYString = dataReadString.Split('@')[1];
                dataX = Convert.ToDouble(dataXString);
                dataY = Convert.ToDouble(dataYString);
                data.Enqueue(new DataPoint(dataX, dataY));
                _ChartValues.Add(new ObservableValue(dataY));
                if (data.Count > PediatricSoftConstants.MaxQueueLength)
                {
                    while (!data.TryDequeue(out dummyDataPoint)) { };
                    _ChartValues.RemoveAt(0);
                }
                LastValue = dataY;
                OnPropertyChanged("LastValue");
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