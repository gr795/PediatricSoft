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
using LiveCharts.Geared;
using System.IO;
using System.Diagnostics;
using System.Timers;

namespace PediatricSoft
{
    public class PediatricSensor : INotifyPropertyChanged
    {

        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private SerialPort _SerialPort;
        private ConcurrentQueue<DataPoint> data = new ConcurrentQueue<DataPoint>();
        private ConcurrentQueue<byte[]> byteStreamQueue = new ConcurrentQueue<byte[]>();
        private bool shouldBeRunning = false;
        private Task streamingTask;
        private Task processingTask;
        private string filePath = String.Empty;
        private StreamWriter file;
        private System.Timers.Timer uiUpdateTimer;

        private byte[] buffer = new byte[PediatricSensorData.ProcessingBufferSize];
        private int bufferIndex = 0;
        private Object bufferLock = new Object();

        public bool ShouldBePlotted { get; set; } = false;
        public GearedValues<ObservableValue> _ChartValues = new GearedValues<ObservableValue>().WithQuality(Quality.Low);
        public int LastValue { get; private set; } = 0;

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

        private void OnUIUpdateTimerEvent(Object source, ElapsedEventArgs e)
        {
            OnPropertyChanged("LastValue");
        }

        private PediatricSensor() { }
        public PediatricSensor(string port)
        {
            Port = port;

            _SerialPort = new SerialPort()
            {
                PortName = port,
                WriteTimeout = PediatricSensorData.SerialPortWriteTimeout,
                ReadTimeout = PediatricSensorData.SerialPortReadTimeout,
                BaudRate = PediatricSensorData.SerialPortBaudRate,
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
                    _SerialPort.DiscardInBuffer();
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
                    _SerialPort.DiscardInBuffer();
                    _SerialPort.Close();
                }
                catch (Exception e) { if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine(e.Message); }

            }
        }

        public void Validate()
        {
            PortOpen();
            if (_SerialPort.IsOpen)
            {
                Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                int bytesToRead = _SerialPort.BytesToRead;
                if (bytesToRead > 0) IsValid = true;
                int randomSN = PediatricSensorData.rnd.Next(1000000);
                SN = randomSN.ToString();
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
                streamingTask = Task.Run(() => StreamData());
                processingTask = Task.Run(() => ProcessData());
                uiUpdateTimer = new System.Timers.Timer(PediatricSensorData.UIUpdateInterval);
                uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
                uiUpdateTimer.Enabled = true;
                if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine($"Started sensor {SN}");
            }
        }

        private void StreamData()
        {
            Stream stream = _SerialPort.BaseStream;
            byte[] byteArrayIn = new byte[PediatricSensorData.SerialPortStreamBlockSize];
            int bytesRead = 0;


            while (shouldBeRunning)
            {
                try
                {
                    bytesRead = stream.Read(byteArrayIn, 0, PediatricSensorData.SerialPortStreamBlockSize);
                }
                catch (Exception e) { if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine(e.Message); }

                if (bytesRead > 0)
                {
                    lock (bufferLock)
                    {
                        Array.Copy(byteArrayIn, 0, buffer, bufferIndex, bytesRead);
                        bufferIndex += bytesRead;
                    }
                    bytesRead = 0;

                }
                Thread.Sleep(PediatricSensorData.rnd.Next(PediatricSensorData.SerialPortStreamSleepMin, PediatricSensorData.SerialPortStreamSleepMax));
            }

        }

        private void ProcessData()
        {
            byte[] data = new byte[PediatricSensorData.DataBlockSize];
            int dataIndex = 0;

            byte[] dataLastValue = new byte[4];

            byte[] localBuffer = new byte[PediatricSensorData.ProcessingBufferSize];
            int localBufferIndex = 0;

            bool inEscape = false;

            if (streamingTask != null)
            {
                while (!streamingTask.IsCompleted)
                {

                    lock (bufferLock)
                    {
                        Array.Copy(buffer, localBuffer, bufferIndex);
                        localBufferIndex = bufferIndex;
                        bufferIndex = 0;
                    }
                    
                    for (int i = 0; i < localBufferIndex; i++)
                    {
                        switch (localBuffer[i])
                        {
                            case PediatricSensorData.FrameEscapeByte:
                                if (inEscape)
                                {
                                    data[dataIndex] = localBuffer[i];
                                    dataIndex++;
                                    inEscape = false;
                                }
                                else inEscape = true;
                                break;

                            case PediatricSensorData.StartDataFrameByte:
                                if (inEscape)
                                {
                                    data[dataIndex] = localBuffer[i];
                                    dataIndex++;
                                    inEscape = false;
                                }
                                else dataIndex = 0;
                                break;

                            default:
                                data[dataIndex] = localBuffer[i];
                                dataIndex++;
                                break;
                        }

                        if (dataIndex == PediatricSensorData.DataBlockSize)
                        {
                            Array.Copy(data, 3, dataLastValue, 1, 3); // copy 24 bits of data
                            dataLastValue[0] = 0; // pad the data with 0x00 to get 32 bits
                            Array.Reverse(dataLastValue); // switch from MSB to LSB
                            LastValue = BitConverter.ToInt32(dataLastValue,0);

                            if (PediatricSensorData.SaveDataEnabled) file.WriteLine(Convert.ToString(LastValue));

                            dataIndex = 0;
                        }
                            

                    }

                    Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                };
            };
        }



        public void Stop()
        {
            if (IsValid)
            {
                shouldBeRunning = false;
                if (streamingTask != null)
                {
                    while (!streamingTask.IsCompleted)
                    {
                        if (PediatricSensorData.IsDebugEnabled) Debug.WriteLine($"Waiting for sensor {SN} to stop");
                    };
                    streamingTask.Dispose();
                };
                PortClose();

                if (uiUpdateTimer != null) uiUpdateTimer.Dispose();
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