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
using System.Collections.Generic;
using System.Linq;

namespace PediatricSoft
{
    public sealed class PediatricSensor : INotifyPropertyChanged, IDisposable
    {

        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private SerialPort _SerialPort;
        private Task streamingTask;
        private Task processingTask;
        private Task stateTask;
        private string filePath = String.Empty;
        private StreamWriter file;
        private System.Timers.Timer uiUpdateTimer;
        private Random rnd = new Random();

        private byte state = PediatricSensorData.SensorStateInit;
        private int sensorADCColdValueRaw;

        private readonly byte[] buffer = new byte[PediatricSensorData.ProcessingBufferSize];
        private int bufferIndex = 0;
        private readonly Object bufferLock = new Object();

        private Queue<int> dataTimeRaw = new Queue<int>(PediatricSensorData.DataQueueLength);
        private Queue<int> dataValueRaw = new Queue<int>(PediatricSensorData.DataQueueLength);

        public GearedValues<ObservableValue> _ChartValues = new GearedValues<ObservableValue>().WithQuality(Quality.Low);
        public bool ShouldBePlotted { get; set; } = false;
        public int LastValue { get; private set; } = 0;
        public int LastTime { get; private set; } = 0;

        public string Port { get; private set; } = String.Empty;
        public string IDN { get; private set; } = String.Empty;
        public string SN { get; private set; } = String.Empty;
        public string PortSN { get { return String.Concat(Port, " - ", SN); } }
        public bool Disposed { get; private set; } = false;
        public bool IsValid
        {
            get
            {
                if (state > PediatricSensorData.SensorStateInit && state < PediatricSensorData.SensorStateShutDown) return true;
                else return false;
            }
        }
        public byte State { get { return state; } }

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
            if (_SerialPort != null)
                if (!_SerialPort.IsOpen)
                {
                    try
                    {
                        _SerialPort.Open();
                        _SerialPort.DiscardOutBuffer();
                        _SerialPort.DiscardInBuffer();
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Opened port {Port}");
                    }
                    catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: {e.Message}"); }
                }
        }

        private void PortClose()
        {
            if (_SerialPort != null)
                if (_SerialPort.IsOpen)
                {
                    try
                    {
                        _SerialPort.DiscardOutBuffer();
                        _SerialPort.DiscardInBuffer();
                        _SerialPort.Close();
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Closed port {Port}");
                    }
                    catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: {e.Message}"); }

                }
        }

        public void Validate()
        {
            PortOpen();

            if (_SerialPort != null)
                if (_SerialPort.IsOpen)
                {
                    try
                    {
                        _SerialPort.WriteLine("@20");
                        _SerialPort.WriteLine("#2");
                        Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        int bytesToRead = _SerialPort.BytesToRead;
                        if (bytesToRead > 0)
                        {
                            state = PediatricSensorData.SensorStateValid;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                            stateTask = Task.Run(() => StateHandler());
                        }
                        else
                        {
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: Not valid. Calling Dispose()");
                            Dispose();
                        }
                    }
                    catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: {e.Message}"); }

                }
                else
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: Not valid. Calling Dispose()");
                    Dispose();
                }
        }

        public void Start()
        {
            if (state == PediatricSensorData.SensorStateIdle)
            {
                filePath = System.IO.Path.Combine(PediatricSensorData.dataFolder, SN);
                filePath += ".txt";
                if (PediatricSensorData.SaveDataEnabled) file = File.AppendText(filePath);

                state++;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

                uiUpdateTimer = new System.Timers.Timer(PediatricSensorData.UIUpdateInterval);
                uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
                uiUpdateTimer.Enabled = true;

            }
        }

        public void Stop()
        {
            if (state == PediatricSensorData.SensorStateRun)
            {

                state = PediatricSensorData.SensorStateStop;
                OnPropertyChanged("State");
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

            while (state >= PediatricSensorData.SensorStateValid && state < PediatricSensorData.SensorStateFailed)
            {
                try
                {
                    bytesRead = stream.Read(byteArrayIn, 0, PediatricSensorData.SerialPortStreamBlockSize);
                }
                catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, e.Message); }

                if (bytesRead > 0)
                {
                    lock (bufferLock)
                    {
                        Array.Copy(byteArrayIn, 0, buffer, bufferIndex, bytesRead);
                        bufferIndex += bytesRead;
                    }
                    bytesRead = 0;

                }
                Thread.Sleep(rnd.Next(PediatricSensorData.SerialPortStreamSleepMin, PediatricSensorData.SerialPortStreamSleepMax));
            }

        }

        private void StreamClose()
        {

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

                            dataTimeRaw.Enqueue(LastTime);
                            if (dataTimeRaw.Count > PediatricSensorData.DataQueueLength) dataTimeRaw.Dequeue();

                            dataValueRaw.Enqueue(LastValue);
                            if (dataValueRaw.Count > PediatricSensorData.DataQueueLength) dataValueRaw.Dequeue();


                            if (ShouldBePlotted)
                            {
                                _ChartValues.Add(new ObservableValue(LastValue));
                                if (_ChartValues.Count > PediatricSensorData.DataQueueLength) _ChartValues.RemoveAt(0);
                            }

                            if (PediatricSensorData.SaveDataEnabled) file.WriteLine(String.Concat(Convert.ToString(LastTime), "\t", Convert.ToString(LastValue)));

                            dataIndex = 0;
                        }

                    }

                    Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                };
            };
        }

        private ushort UShortSafeInc(ushort value, ushort step, ushort max)
        {
            int newValue = value + step;
            if (newValue < max)
                return (ushort)newValue;
            else return max;
        }

        private ushort UShortSafeDec(ushort value, ushort step, ushort min)
        {
            int newValue = value - step;
            if (newValue > min)
                return (ushort)newValue;
            else return min;
        }



        private void SendCommandsIdle()
        {
            SendCommand(PediatricSensorData.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorIdleLaserCurrent)));

            SendCommand(PediatricSensorData.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorIdleLaserHeat)));

            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorIdleCellHeat)));
        }

        private void SendCommandsLaserLock()
        {

            bool foundResonanceSweep = false;
            int numberOfLaserLockSweepCycles = 0;
            int numberOfLaserLockStepCycles = 0;
            int[] data = new int[PediatricSensorData.DataQueueLength];
            double transmission = 0;

            // Turn on the laser
            SendCommand(PediatricSensorData.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorDefaultLaserCurrent)));

            // Wait a bit and record the ADC value
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerADCColdDelay} ms");
            Thread.Sleep(PediatricSensorData.StateHandlerADCColdDelay);
            sensorADCColdValueRaw = LastValue;

            // Set the cell heater to the max value, wait for the cell to warm up
            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorMaxCellHeat)));

            // Laser heat sweep cycle. Here we look for the Rb resonance.
            while (!foundResonanceSweep && state == PediatricSensorData.SensorStateLaserLock && numberOfLaserLockSweepCycles < PediatricSensorData.MaxNumberOfLaserLockSweepCycles)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Laser lock sweep cycle: {numberOfLaserLockSweepCycles}");

                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerCellHeatInitialTime} ms");
                Thread.Sleep(PediatricSensorData.StateHandlerCellHeatInitialTime);

                // Set the laser heater to the max value, wait a bit
                SendCommand(PediatricSensorData.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorMaxLaserHeat)));
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerLaserHeatSweepTime} ms");
                Thread.Sleep(PediatricSensorData.StateHandlerLaserHeatSweepTime);

                if (dataValueRaw.Count >= PediatricSensorData.DataQueueLength)
                {

                    // See if the threshold was reached
                    data = dataValueRaw.ToArray();
                    transmission = (double)data.Min() / sensorADCColdValueRaw;
                    if (transmission < PediatricSensorData.SensorTargetLaserTransmission)
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Found Rb resonance");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Transmission on resonance: {transmission}");
                        foundResonanceSweep = true;
                        SendCommand(PediatricSensorData.SensorCommandLaserHeat);
                        SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorMinLaserHeat)));

                    }
                    else if (numberOfLaserLockSweepCycles < PediatricSensorData.MaxNumberOfLaserLockSweepCycles)
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, going to wait more");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Minimal transmission {transmission} is higher than the target {PediatricSensorData.SensorTargetLaserTransmission}");
                        SendCommand(PediatricSensorData.SensorCommandLaserHeat);
                        SendCommand(String.Concat("#", PediatricSensorData.UInt16ToStringBE(PediatricSensorData.SensorMinLaserHeat)));
                        numberOfLaserLockSweepCycles++;
                    }

                }
                else
                {
                    SendCommandsIdle();
                    state = PediatricSensorData.SensorStateFailed;
                    OnPropertyChanged("State");
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                }
            }

            if (foundResonanceSweep)
            {

                while (state == PediatricSensorData.SensorStateLaserLock && numberOfLaserLockStepCycles < PediatricSensorData.MaxNumberOfLaserLockStepCycles)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Laser lock sweep cycle: {numberOfLaserLockStepCycles}");


                    numberOfLaserLockStepCycles++;
                }

            }
            else
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, giving up");
                SendCommandsIdle();
                state = PediatricSensorData.SensorStateFailed;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }

        }

        private void StateHandler()
        {

            while (state > PediatricSensorData.SensorStateInit && state < PediatricSensorData.SensorStateFailed)
            {
                switch (state)
                {

                    case PediatricSensorData.SensorStateValid:
                        SendCommandsIdle();
                        streamingTask = Task.Run(() => StreamData());
                        processingTask = Task.Run(() => ProcessData());
                        if (state == PediatricSensorData.SensorStateValid)
                        {
                            state++;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateIdle:
                        break;

                    case PediatricSensorData.SensorStateStart:

                        if (state == PediatricSensorData.SensorStateStart)
                        {
                            state++;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLock:

                        SendCommandsLaserLock();
                        if (state == PediatricSensorData.SensorStateLaserLock)
                        {
                            state++;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateRun:
                        break;

                    case PediatricSensorData.SensorStateStop:
                        SendCommandsIdle();
                        if (state == PediatricSensorData.SensorStateStop)
                        {
                            state = PediatricSensorData.SensorStateIdle;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateFailed:
                        break;

                    case PediatricSensorData.SensorStateShutDown:
                        break;

                    default:
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Unknown state. We shouldn't be here.");
                        break;
                }

                Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
            }

            StreamClose();

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: State Handler exiting.");
        }



        public void SendCommand(string command)
        {

            command = Regex.Replace(command, @"[^\w@#]", "");

            if (state > PediatricSensorData.SensorStateInit && state < PediatricSensorData.SensorStateShutDown)
            {
                if (!String.IsNullOrEmpty(command))
                {
                    if (_SerialPort != null)
                        if (_SerialPort.IsOpen)
                        {
                            try
                            {
                                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: sending command \"{command}\"");
                                _SerialPort.WriteLine(command);
                            }
                            catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, e.Message); }
                        }

                }
            }
            else
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Can't send commands - sensor not running");
        }

        public void Dispose()
        {
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Dispose() was called");

            if (state > PediatricSensorData.SensorStateInit)
            {
                state = PediatricSensorData.SensorStateShutDown;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }

            if (stateTask != null)
            {
                while (!stateTask.IsCompleted)
                {
                    Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
                };
                stateTask.Dispose();
            };

            if (_SerialPort != null)
            {
                PortClose();
                _SerialPort.Dispose();
            };

            Disposed = true;
        }

        ~PediatricSensor()
        {
            if (!Disposed) Dispose();
        }

    }
}