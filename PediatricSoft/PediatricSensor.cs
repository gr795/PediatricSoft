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
using System.Timers;

namespace PediatricSoft
{
    public class PediatricSensor : INotifyPropertyChanged
    {

        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private SerialPort _SerialPort;
        private Task streamingTask;
        private Task processingTask;
        private Task stateTask;
        private string filePath = String.Empty;
        private StreamWriter file;
        private System.Timers.Timer uiUpdateTimer;

        private byte state = PediatricSensorData.SensorStateInit;

        private byte[] buffer = new byte[PediatricSensorData.ProcessingBufferSize];
        private int bufferIndex = 0;
        private Object bufferLock = new Object();

        public bool ShouldBePlotted { get; set; } = false;
        public GearedValues<ObservableValue> _ChartValues = new GearedValues<ObservableValue>().WithQuality(Quality.Low);
        public int LastValue { get; private set; } = 0;
        public int LastTime { get; private set; } = 0;

        public string Port { get; private set; } = String.Empty;
        public string IDN { get; private set; } = String.Empty;
        public string SN { get; private set; } = String.Empty;
        public string PortSN { get { return String.Concat(Port, " - ", SN); } }
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
            SN = Regex.Replace(port, @"[^\d]", "");

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
                catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled,e.Message); }
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
                catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled,e.Message); }

            }
        }

        public void Validate()
        {
            PortOpen();
            if (_SerialPort.IsOpen)
            {
                _SerialPort.WriteLine("@20");
                _SerialPort.WriteLine("#2");
                Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                int bytesToRead = _SerialPort.BytesToRead;
                if (bytesToRead > 0)
                {
                    IsValid = true;
                    state = PediatricSensorData.SensorStateValid;
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                    StateHandler();
                }
                
                PortClose();
            }
        }

        public void Start()
        {
            if (state == PediatricSensorData.SensorStateIdle)
            {
                filePath = System.IO.Path.Combine(PediatricSensorData.dataFolder, SN);
                filePath += ".txt";
                if (PediatricSensorData.SaveDataEnabled) file = File.AppendText(filePath);
                
                state = PediatricSensorData.SensorStateLaserLock;
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

                stateTask = Task.Run(() => StateHandler());

                uiUpdateTimer = new System.Timers.Timer(PediatricSensorData.UIUpdateInterval);
                uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
                uiUpdateTimer.Enabled = true;
                
            }
        }

        public void Stop()
        {
            if (state == PediatricSensorData.SensorStateRun)
            {

                state = PediatricSensorData.SensorStateShutDown;
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

                if (uiUpdateTimer != null) uiUpdateTimer.Dispose();
                if (PediatricSensorData.SaveDataEnabled && (file != null)) file.Dispose();
                
            }
        }

        private void StreamData()
        {
            Stream stream = _SerialPort.BaseStream;
            byte[] byteArrayIn = new byte[PediatricSensorData.SerialPortStreamBlockSize];
            int bytesRead = 0;


            while (state > PediatricSensorData.SensorStateIdle && state < PediatricSensorData.SensorStateShutDown)
            {
                try
                {
                    bytesRead = stream.Read(byteArrayIn, 0, PediatricSensorData.SerialPortStreamBlockSize);
                }
                catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled,e.Message); }

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
                            Array.Reverse(data); // switch from MSB to LSB
                            LastTime = BitConverter.ToInt32(data, 4);
                            LastValue = BitConverter.ToInt32(data, 0);

                            if (ShouldBePlotted)
                            {
                                _ChartValues.Add(new ObservableValue(LastValue));
                                if (_ChartValues.Count > PediatricSensorData.MaxQueueLength) _ChartValues.RemoveAt(0);
                            }

                            if (PediatricSensorData.SaveDataEnabled) file.WriteLine( String.Concat( Convert.ToString(LastTime), "\t", Convert.ToString(LastValue)));

                            dataIndex = 0;
                        }

                    }

                    Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                };
            };
        }

        private void StateHandler()
        {

            void SendIdleCommands()
            {
                SendCommand(PediatricSensorData.SensorCommandCurrent);
                SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorIdleCurrent)));
            }

            void SendLaserLockCommands()
            {
                SendCommand(PediatricSensorData.SensorCommandCurrent);
                SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorDefaultCurrent)));
            }

            while (state > PediatricSensorData.SensorStateInit)
            {
                switch (state)
                {

                    case PediatricSensorData.SensorStateValid:
                        PortOpen();
                        SendIdleCommands();
                        PortClose();
                        state++;
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateIdle:
                        break;

                    case PediatricSensorData.SensorStateLaserLock:
                        PortOpen();
                        streamingTask = Task.Run(() => StreamData());
                        processingTask = Task.Run(() => ProcessData());
                        SendLaserLockCommands();
                        state++;
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateRun:
                        break;

                    case PediatricSensorData.SensorStateShutDown:
                        SendIdleCommands();
                        if (streamingTask != null)
                        {
                            while (!streamingTask.IsCompleted)
                            {
                                Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
                            };
                            streamingTask.Dispose();
                        };
                        if (processingTask != null)
                        {
                            while (!processingTask.IsCompleted)
                            {
                                Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
                            };
                            processingTask.Dispose();
                        };
                        PortClose();
                        state = PediatricSensorData.SensorStateIdle;
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    default:
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Unknown state. We shouldn't be here.");
                        break;
                }

                if (state == PediatricSensorData.SensorStateIdle) break;
                else
                    Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
            }

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: State Handler exiting.");
        }

        

        public void SendCommand(string command)
        {

            command = Regex.Replace(command, @"[^\w@#]", "");

            if (state > PediatricSensorData.SensorStateInit && state < PediatricSensorData.SensorStateLast)
            {
                if (!String.IsNullOrEmpty(command))
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: sending command \"{command}\"");
                    _SerialPort.WriteLine(command);
                }
            }
            else
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Can't send commands - sensor failed");
        }

        ~PediatricSensor()
        {
            Stop();
            _SerialPort.Dispose();
        }

    }
}