using FTD2XX_NET;
using LiveCharts;
using LiveCharts.Defaults;
using Prism.Mvvm;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using System.Xml.Serialization;

namespace PediatricSoft
{
    public sealed class PediatricSensor : BindableBase, IDisposable
    {

        private PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private PediatricSensorConfig PediatricSensorConfig;
        private PediatricSensorConfig PediatricSensorConfigOnLoad;
        private readonly string configPath;

        private FTDI _FTDI;
        private Task streamingTask;
        private Task stateHandlerTask;
        private System.Timers.Timer uiUpdateTimer;

        private readonly Object stateLock = new Object();
        private PediatricSoftConstants.SensorState state = PediatricSoftConstants.SensorState.Init;
        public PediatricSoftConstants.SensorState State
        {
            get { return state; }
            private set
            {
                state = value;
                RaisePropertyChanged();
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {value}");
            }
        }

        private int sensorADCColdValueRaw;
        private double sensorZDemodCalibration = 1;
        private ushort zeroXField = PediatricSoftConstants.SensorColdFieldXOffset;
        private ushort zeroYField = PediatricSoftConstants.SensorColdFieldYOffset;
        private ushort zeroZField = PediatricSoftConstants.SensorColdFieldZOffset;

        private bool infoRequested = false;
        private string requestedInfoString = string.Empty;

        //private readonly Object commandLock = new Object();
        //private bool commandSent = false;
        private ConcurrentQueue<string> commandQueue = new ConcurrentQueue<string>();

        private readonly Object dataLock = new Object();
        private Queue<int> dataTimeRaw = new Queue<int>(PediatricSoftConstants.DataQueueLength);
        private Queue<int> dataValueRaw = new Queue<int>(PediatricSoftConstants.DataQueueLength);

        public ChartValues<ObservableValue> ChartValues = new ChartValues<ObservableValue>();

        private bool isPlotted = false;
        public bool IsPlotted
        {
            get { return isPlotted; }
            set { isPlotted = value; RaisePropertyChanged(); PediatricSensorData.UpdateSeriesCollection(); }
        }

        private volatile int lastTimeRAW = 0;
        public int LastTimeRAW { get { return lastTimeRAW; } }

        private volatile int lastValueRAW = 0;
        public int LastValueRAW { get { return lastValueRAW; } }

        public double LastValue { get; private set; } = 0;
        private volatile bool dataUpdated = false;

        public string Port { get; private set; } = String.Empty;
        public string SN { get; private set; } = String.Empty;
        public string PortSN { get { return String.Concat(Port, " - ", SN); } }
        public bool CanSendCommand { get; private set; } = true;
        public bool IsDisposed { get; private set; } = false;
        public bool IsLocked { get; private set; } = false;
        public bool IsRunning { get; private set; } = false;
        public bool IsValid { get; private set; } = false;


        private void OnUIUpdateTimerEvent(Object source, ElapsedEventArgs e)
        {
            RaisePropertyChanged("LastValueRAW");
            RaisePropertyChanged("LastValue");
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

        private string UInt16ToStringBE(ushort value)
        {
            byte[] t = BitConverter.GetBytes(value);
            Array.Reverse(t);
            string s = BitConverter.ToString(t);
            s = Regex.Replace(s, @"[^\w]", "");
            return s;
        }

        private PediatricSensor() { }
        public PediatricSensor(string serial)
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            _FTDI = new FTDI();

            SN = serial;

            configPath = System.IO.Path.Combine(PediatricSensorData.Instance.SensorConfigFolderAbsolute, SN) + ".xml";
            LoadConfig();

            // Open the device by serial number
            ftStatus = _FTDI.OpenBySerialNumber(serial);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to open FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set Baud rate
            ftStatus = _FTDI.SetBaudRate(PediatricSoftConstants.SerialPortBaudRate);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set Baud rate on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set data characteristics - Data bits, Stop bits, Parity
            ftStatus = _FTDI.SetDataCharacteristics(FTDI.FT_DATA_BITS.FT_BITS_8, FTDI.FT_STOP_BITS.FT_STOP_BITS_1, FTDI.FT_PARITY.FT_PARITY_NONE);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set data characteristics on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set flow control - set RTS/CTS flow control - we don't have flow control
            ftStatus = _FTDI.SetFlowControl(FTDI.FT_FLOW_CONTROL.FT_FLOW_NONE, 0x11, 0x13);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set flow control on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set timeouts
            ftStatus = _FTDI.SetTimeouts(PediatricSoftConstants.SerialPortReadTimeout, PediatricSoftConstants.SerialPortWriteTimeout);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set timeouts on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set latency
            ftStatus = _FTDI.SetLatency(PediatricSoftConstants.SerialPortLatency);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set latency on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Get COM Port Name
            ftStatus = _FTDI.GetCOMPort(out string port);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to get the COM Port Name on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }
            Port = port;
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Opened Port {port} on FTDI device with S/N {SN}");

            // Purge port buffers
            ftStatus = _FTDI.Purge(FTDI.FT_PURGE.FT_PURGE_RX + FTDI.FT_PURGE.FT_PURGE_TX);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to purge buffers on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            const string checkString = "?\n";
            UInt32 numBytes = 0;

            // Write ? into the port to see if we have a working board
            ftStatus = _FTDI.Write(checkString, checkString.Length, ref numBytes);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to write to FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Wait a bit
            Thread.Sleep((int)PediatricSoftConstants.SerialPortWriteTimeout);

            // Check if the board responded
            ftStatus = _FTDI.GetRxBytesAvailable(ref numBytes);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Purge port buffers
            ftStatus = _FTDI.Purge(FTDI.FT_PURGE.FT_PURGE_RX + FTDI.FT_PURGE.FT_PURGE_TX);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to purge buffers on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            if (numBytes > 0)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Valid");
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Valid;
                }

                IsValid = true;
            }
            else
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't get a response from the board: not valid");
                Dispose();
            }

        }

        public void KickOffTasks()
        {
            streamingTask = StreamDataAsync();
            stateHandlerTask = StateHandlerAsync();

            uiUpdateTimer = new System.Timers.Timer(PediatricSoftConstants.UIUpdateInterval);
            uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
            uiUpdateTimer.Enabled = true;
        }

        public void Lock()
        {
            PediatricSoftConstants.SensorState currentState;
            bool canRun = false;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.Valid ||
                    currentState == PediatricSoftConstants.SensorState.LaserLockDone ||
                    currentState == PediatricSoftConstants.SensorState.Idle)
                {
                    State = PediatricSoftConstants.SensorState.Setup;
                    currentState = State;
                    canRun = true;
                }


            }

            if (canRun)
                while (currentState != PediatricSoftConstants.SensorState.LaserLockDone ||
                       currentState == PediatricSoftConstants.SensorState.Failed)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }
        }

        public void Start()
        {
            PediatricSoftConstants.SensorState currentState;
            bool canRun = false;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.Idle)
                {
                    State++;
                    currentState = State;
                    canRun = true;
                }
            }

            if (canRun)
                while (currentState != PediatricSoftConstants.SensorState.Run || currentState == PediatricSoftConstants.SensorState.Failed)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }

        }

        public void Stop()
        {
            PediatricSoftConstants.SensorState currentState;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.Run)
                    State++;
            }

            if (currentState == PediatricSoftConstants.SensorState.Run)
                while (currentState != PediatricSoftConstants.SensorState.Idle || currentState == PediatricSoftConstants.SensorState.Failed)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }
        }

        public void ZeroFields()
        {
            PediatricSoftConstants.SensorState currentState;
            bool canRun = false;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.LaserLockDone ||
                    currentState == PediatricSoftConstants.SensorState.Idle)
                {
                    State = PediatricSoftConstants.SensorState.ZeroFields;
                    currentState = State;
                    canRun = true;
                }
            }

            if (canRun)
                while (currentState != PediatricSoftConstants.SensorState.Idle || currentState == PediatricSoftConstants.SensorState.Failed)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }
        }

        private Task StreamDataAsync()
        {
            return Task.Run(() =>
            {
                PediatricSoftConstants.SensorState currentState;
                lock (stateLock)
                {
                    currentState = State;
                }

                FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

                byte[] streamingBuffer = new byte[PediatricSoftConstants.StreamingBufferSize];
                UInt32 bytesToRead = 0;
                UInt32 bytesRead = 0;
                UInt32 bytesWritten = 0;

                byte currentByte;
                string command = string.Empty;

                byte[] data = new byte[PediatricSoftConstants.DataBlockSize];
                UInt32 dataIndex = 0;

                byte[] info = new byte[PediatricSoftConstants.InfoBlockSize];
                UInt32 infoIndex = 0;

                bool inEscape = false;
                bool inInfoFrame = false;

                bool dataSaveEnable = false;
                bool dataSaveRAW = true;

                List<string> dataSaveBuffer = new List<string>();
                string filePath = string.Empty;

                int plotCounter = 0;
                const int plotCounterMax = PediatricSoftConstants.DataQueueLength / PediatricSoftConstants.PlotQueueLength;

                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Streaming starting");

                while (currentState < PediatricSoftConstants.SensorState.ShutDownComplete)
                {
                    // First we write out all of the queued commands
                    while (commandQueue.TryDequeue(out command))
                    {
                        ftStatus = _FTDI.Write(command, command.Length, ref bytesWritten);
                        if (ftStatus != FTDI.FT_STATUS.FT_OK)
                        {
                            lock (stateLock)
                            {
                                State = PediatricSoftConstants.SensorState.Failed;
                            }
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to write to FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                            break;
                        }
                    }

                    // Query if we have bytes in the Rx buffer
                    if (ftStatus == FTDI.FT_STATUS.FT_OK)
                        ftStatus = _FTDI.GetRxBytesAvailable(ref bytesToRead);
                    if (ftStatus != FTDI.FT_STATUS.FT_OK)
                    {
                        lock (stateLock)
                        {
                            State = PediatricSoftConstants.SensorState.Failed;
                        }
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                        break;
                    }

                    // We read up to a maximum of the streaming buffer
                    if (bytesToRead > PediatricSoftConstants.StreamingBufferSize)
                        bytesToRead = PediatricSoftConstants.StreamingBufferSize;

                    // Read bytes from the COM port.
                    // This thing blocks!
                    // It will return when either it has read the requested number of bytes or after the ReadTimeout time.
                    // It will update the bytesRead value with the actual number of bytes read.
                    if (ftStatus == FTDI.FT_STATUS.FT_OK && bytesToRead > 0)
                        ftStatus = _FTDI.Read(streamingBuffer, bytesToRead, ref bytesRead);
                    if (ftStatus != FTDI.FT_STATUS.FT_OK)
                    {
                        lock (stateLock)
                        {
                            State = PediatricSoftConstants.SensorState.Failed;
                        }
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to read in the Rx buffer on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                        break;
                    }

                    // If we read more than 0 bytes, we run the processing logic on the buffer and reset the bytesRead at the end.
                    if (bytesRead > 0)
                    {
                        // Check if we need to save data
                        lock (stateLock)
                        {
                            if (State == PediatricSoftConstants.SensorState.Start)
                            {
                                filePath = System.IO.Path.Combine(PediatricSensorData.SaveFolderCurrentRun, SN);
                                filePath += ".txt";

                                dataSaveBuffer.Clear();

                                dataSaveEnable = PediatricSensorData.SaveDataEnabled;
                                dataSaveRAW = PediatricSensorData.SaveRAWValues;

                                State++;
                            }
                        }

                        // This loop sorts bytes into appropriate buffers
                        for (int i = 0; i < (int)bytesRead; i++)
                        {
                            currentByte = Buffer.GetByte(streamingBuffer, i);
                            switch (currentByte)
                            {

                                case PediatricSoftConstants.FrameEscapeByte:
                                    if (inEscape)
                                    {
                                        data[dataIndex] = currentByte;
                                        dataIndex++;
                                        inEscape = false;
                                    }
                                    else inEscape = true;
                                    break;

                                case PediatricSoftConstants.StartDataFrameByte:
                                    if (inEscape)
                                    {
                                        data[dataIndex] = currentByte;
                                        dataIndex++;
                                        inEscape = false;
                                    }
                                    else
                                    {
                                        inInfoFrame = false;
                                        infoIndex = 0;
                                        dataIndex = 0;
                                    }
                                    break;

                                case PediatricSoftConstants.StartInfoFrameByte:
                                    if (inEscape)
                                    {
                                        data[dataIndex] = currentByte;
                                        dataIndex++;
                                        inEscape = false;
                                    }
                                    else
                                    {
                                        inInfoFrame = true;
                                        infoIndex = 0;
                                        dataIndex = 0;
                                    }
                                    break;

                                default:
                                    if (inInfoFrame)
                                    {
                                        info[infoIndex] = currentByte;
                                        infoIndex++;
                                    }
                                    else
                                    {
                                        data[dataIndex] = currentByte;
                                        dataIndex++;
                                    }
                                    break;
                            }

                            // If we have enought of info bytes - process them
                            if (infoIndex == PediatricSoftConstants.InfoBlockSize)
                            {
                                string infoMessage = System.Text.Encoding.ASCII.GetString(info);
                                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Info message: {infoMessage}");
                                PediatricSensorData.CommandHistory = String.Concat(infoMessage, "\n", PediatricSensorData.CommandHistory);
                                lock (dataLock)
                                {
                                    if (infoRequested)
                                    {
                                        requestedInfoString = infoMessage;
                                        infoRequested = false;
                                    }
                                }
                                inInfoFrame = false;
                                infoIndex = 0;
                            }

                            // If we have enought of data bytes - process them
                            if (dataIndex == PediatricSoftConstants.DataBlockSize)
                            {
                                lock (dataLock)
                                {
                                    Array.Reverse(data); // switch from MSB to LSB
                                    lastTimeRAW = BitConverter.ToInt32(data, 4);
                                    lastValueRAW = BitConverter.ToInt32(data, 0);
                                    LastValue = lastValueRAW * PediatricSoftConstants.SensorADCRawToVolts;

                                    dataTimeRaw.Enqueue(lastTimeRAW);
                                    if (dataTimeRaw.Count > PediatricSoftConstants.DataQueueLength) dataTimeRaw.Dequeue();

                                    dataValueRaw.Enqueue(lastValueRAW);
                                    if (dataValueRaw.Count > PediatricSoftConstants.DataQueueLength) dataValueRaw.Dequeue();

                                    if (dataSaveEnable)
                                        if (dataSaveRAW)
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(lastTimeRAW), "\t", Convert.ToString(lastValueRAW)));
                                        else
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(lastTimeRAW * 0.001), "\t", Convert.ToString(lastValueRAW * sensorZDemodCalibration)));

                                    if (!dataUpdated) dataUpdated = true;
                                }

                                if (IsPlotted)
                                {
                                    plotCounter++;
                                    if (plotCounter == plotCounterMax)
                                    {
                                        ChartValues.Add(new ObservableValue(LastValue));
                                        if (ChartValues.Count > PediatricSoftConstants.PlotQueueLength) ChartValues.RemoveAt(0);
                                        plotCounter = 0;
                                    }
                                }

                                dataIndex = 0;
                            }

                        }

                        // Check if we need to stop saving
                        lock (stateLock)
                        {
                            if (State > PediatricSoftConstants.SensorState.Run)
                            {
                                if (!string.IsNullOrEmpty(filePath))
                                    File.AppendAllLines(filePath, dataSaveBuffer);

                                filePath = string.Empty;
                                dataSaveBuffer.Clear();

                                dataSaveEnable = false;
                                dataSaveRAW = true;

                                if (State == PediatricSoftConstants.SensorState.Stop)
                                    State = PediatricSoftConstants.SensorState.Idle;
                            }
                        }

                        // Reset bytesRead to 0
                        bytesRead = 0;

                    }

                    // Sleep a bit
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);

                    // Update the current State
                    lock (stateLock)
                    {
                        currentState = State;
                    }

                }

                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Streaming stoping");

            });
        }

        private void SendCommandsSetup()
        {
            SendCommand(PediatricSoftConstants.SensorCommandLaserLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockDisable)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDInverse);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDInverse)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdLaserCurrent)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrentModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserCurrentModulation)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdLaserHeat)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldXOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdFieldXOffset)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldXModulationAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdFieldXModulationAmplitude)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldYOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdFieldYOffset)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldYModulationAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdFieldYModulationAmplitude)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdFieldZOffset)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldZModulationAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdFieldZModulationAmplitude)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserModulationFrequency);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserModulationFrequency)));

            SendCommand(PediatricSoftConstants.SensorCommandDelayForLaser);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultDelayForLaser)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDLaserCurrentI);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDLaserCurrentI)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDLaserHeaterI);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDLaserHeaterI)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDCellHeaterI);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDCellHeaterI)));

            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndADCGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataStreamingOnGainLow)));

            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdCellHeat)));

            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataSelectorADC)));

            SendCommand(PediatricSoftConstants.SensorCommandADCDisplayGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultADCDisplayGain)));

            SendCommand(PediatricSoftConstants.SensorCommand2fPhase);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefault2fPhase)));

            SendCommand(PediatricSoftConstants.SensorCommandBzPhase);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultBzPhase)));

            lock (stateLock)
            {
                if (State == PediatricSoftConstants.SensorState.Setup)
                    State++;
            }
        }

        private void SendCommandsLaserLockSweep()
        {
            PediatricSoftConstants.SensorState currentState;
            PediatricSoftConstants.SensorState correctState;

            lock (stateLock)
            {
                currentState = State;
                correctState = State;
            }

            int[] data = new int[PediatricSoftConstants.DataQueueLength];
            double transmission = double.MaxValue;

            // Turn on the laser
            lock (dataLock)
            {
                SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorConfig.LaserCurrent)));
                dataUpdated = false;
            }

            // Wait a bit
            Thread.Sleep(PediatricSoftConstants.StateHandlerLaserHeatSweepTime);

            // Record the ADC value
            while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
            sensorADCColdValueRaw = lastValueRAW;

            // Here we check if we received non-zero value.
            // If it is zero (default) - something is wrong and we fail.
            if (sensorADCColdValueRaw == 0)
            {
                State = PediatricSoftConstants.SensorState.Failed;
                return;
            }

            // Set the cell heater to the max value
            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorConfig.MaxCellHeat)));

            // Wait for the cell to warm up
            Thread.Sleep(PediatricSoftConstants.StateHandlerCellHeatInitialTime);

            // Turn on the Z-coil to increase absorption
            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockFieldZOffset)));

            // Laser heat sweep cycle. Here we look for the Rb resonance.

            for (int i = 0; i < PediatricSoftConstants.MaxNumberOfLaserLockSweepCycles; i++)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Laser lock sweep cycle: {i}");

                // Set the laser heater to the max value
                SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMaxLaserHeat)));

                // Wait a bit
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Waiting for {PediatricSoftConstants.StateHandlerLaserHeatSweepTime} ms");
                Thread.Sleep(PediatricSoftConstants.StateHandlerLaserHeatSweepTime);

                // Set the laser heater to the min value
                SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMinLaserHeat)));

                // Wait a bit
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Waiting for {PediatricSoftConstants.StateHandlerLaserHeatSweepTime} ms");
                Thread.Sleep(PediatricSoftConstants.StateHandlerLaserHeatSweepTime);

                // See if the threshold was reached
                lock (dataLock)
                {
                    data = dataValueRaw.ToArray();
                }
                transmission = (double)data.Min() / sensorADCColdValueRaw;

                if (transmission < PediatricSoftConstants.SensorTargetLaserTransmissionSweep)
                {
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Sweep cycle - found Rb resonance");
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Transmission on resonance: {transmission}");

                    lock (stateLock)
                    {
                        currentState = State;
                        if (currentState == correctState)
                        {
                            State++;
                        }
                    }
                    return;
                }
                else
                {
                    // Check if the lock was cancelled
                    lock (stateLock)
                    {
                        currentState = State;
                        if (currentState != correctState)
                        {
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Lock was aborted or something failed. Returning.");
                            return;
                        }
                    }
                }

                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, going to wait more");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Minimal transmission {transmission} is higher than the target {PediatricSoftConstants.SensorTargetLaserTransmissionSweep}");

            }

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, giving up");

            // Fail
            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                    return;
                }
            }

        }

        private void SendCommandsLaserLockStep()
        {
            PediatricSoftConstants.SensorState currentState;
            PediatricSoftConstants.SensorState correctState;

            lock (stateLock)
            {
                currentState = State;
                correctState = State;
            }

            ushort laserHeat = PediatricSoftConstants.SensorMinLaserHeat;
            double transmission = double.MaxValue;

            lock (dataLock)
            {
                SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)));
                dataUpdated = false;
            }

            for (int i = 0; i < PediatricSoftConstants.MaxNumberOfLaserLockStepCycles; i++)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Laser lock step cycle: {i}");

                while (laserHeat < PediatricSoftConstants.SensorMaxLaserHeat)
                {
                    while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                    transmission = (double)lastValueRAW / sensorADCColdValueRaw;

                    if (transmission < PediatricSoftConstants.SensorTargetLaserTransmissionStep)
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Step cycle - found Rb resonance");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Transmission on resonance: {transmission}");

                        lock (stateLock)
                        {
                            currentState = State;
                            if (currentState == correctState)
                            {
                                State++;
                            }
                        }
                        return;
                    }
                    else
                    {
                        // Check if the lock was cancelled
                        lock (stateLock)
                        {
                            currentState = State;
                            if (currentState != correctState)
                            {
                                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Lock was aborted or something failed. Returning.");
                                return;
                            }
                        }
                    }

                    laserHeat = UShortSafeInc(laserHeat, PediatricSoftConstants.SensorLaserHeatStep, PediatricSoftConstants.SensorMaxLaserHeat);
                    lock (dataLock)
                    {
                        SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)));
                        dataUpdated = false;
                    }

                }

                // Set laser heat to min for the next cycle
                laserHeat = PediatricSoftConstants.SensorMinLaserHeat;

            }

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: We saw Rb resonance, but the step cycle failed. Giving up");
                    State = PediatricSoftConstants.SensorState.Failed;
                }
            }

        }

        private void SendCommandsLaserLockPID()
        {
            PediatricSoftConstants.SensorState currentState;
            PediatricSoftConstants.SensorState correctState;

            lock (stateLock)
            {
                currentState = State;
                correctState = State;
            }

            //SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndGain);
            //SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataStreamingOnGainHigh)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockEnable)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserHeat)));

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State++;
                }
                else
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Lock was aborted or something failed. Returning.");
            }
        }

        private void SendCommandsStabilizeCellHeat()
        {
            PediatricSoftConstants.SensorState currentState;
            PediatricSoftConstants.SensorState correctState;

            lock (stateLock)
            {
                currentState = State;
                correctState = State;
            }

            double transmission = 0;

            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdCellHeat)));

            while (currentState == correctState && transmission < PediatricSoftConstants.SensorTargetLaserTransmissionSweep)
            {
                Thread.Sleep(PediatricSoftConstants.SensorLaserHeatStepSleepTime);
                lock (dataLock)
                {
                    transmission = (double)lastValueRAW / sensorADCColdValueRaw;
                }
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultCellHeat)));

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State++;
                }
                else
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Lock was aborted or something failed. Returning.");
            }
        }

        private void SendCommandsZeroFields()
        {
            int reminder = 0;
            ushort plusField = ushort.MaxValue;
            ushort minusField = ushort.MinValue;
            double plusValue = 0;
            double minusValue = 0;

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Begin field zeroing");

            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataSelectorADC)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldZModulationAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdFieldZModulationAmplitude)));

            for (int i = 0; i < (PediatricSoftConstants.NumberOfFieldZeroingSteps * 3); i++)
            {
                Math.DivRem(i, 3, out reminder);
                switch (reminder)
                {
                    case -1:
                        SendCommand(PediatricSoftConstants.SensorCommandFieldXOffset);
                        plusField = UShortSafeInc(zeroXField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroXField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)));
                        lock (dataLock)
                        {
                            dataUpdated = false;
                        }
                        while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (dataLock)
                        {
                            plusValue = (double)lastValueRAW;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)));
                        lock (dataLock)
                        {
                            dataUpdated = false;
                        }
                        while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (dataLock)
                        {
                            minusValue = (double)lastValueRAW;
                        }

                        if (plusValue > minusValue)
                            zeroXField = UShortSafeInc(zeroXField, PediatricSoftConstants.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroXField = UShortSafeDec(zeroXField, PediatricSoftConstants.SensorFieldStep, ushort.MinValue);

                        break;

                    case 1:
                        SendCommand(PediatricSoftConstants.SensorCommandFieldYOffset);
                        plusField = UShortSafeInc(zeroYField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroYField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)));
                        lock (dataLock)
                        {
                            dataUpdated = false;
                        }
                        while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (dataLock)
                        {
                            plusValue = (double)lastValueRAW;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)));
                        lock (dataLock)
                        {
                            dataUpdated = false;
                        }
                        while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (dataLock)
                        {
                            minusValue = (double)lastValueRAW;
                        }

                        if (plusValue > minusValue)
                            zeroYField = UShortSafeInc(zeroYField, PediatricSoftConstants.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroYField = UShortSafeDec(zeroYField, PediatricSoftConstants.SensorFieldStep, ushort.MinValue);

                        break;

                    case 2:
                        SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
                        plusField = UShortSafeInc(zeroZField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroZField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)));
                        lock (dataLock)
                        {
                            dataUpdated = false;
                        }
                        while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (dataLock)
                        {
                            plusValue = (double)lastValueRAW;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)));
                        lock (dataLock)
                        {
                            dataUpdated = false;
                        }
                        while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (dataLock)
                        {
                            minusValue = (double)lastValueRAW;
                        }

                        if (plusValue > minusValue)
                            zeroZField = UShortSafeInc(zeroZField, PediatricSoftConstants.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroZField = UShortSafeDec(zeroZField, PediatricSoftConstants.SensorFieldStep, ushort.MinValue);

                        break;

                    default:
                        break;
                }
            }

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Field zeroing done");
        }

        private void SendCommandsCalibrateMagnetometer()
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Begin calibration");

            ushort plusField = UShortSafeInc(zeroZField, PediatricSoftConstants.SensorFieldStep, ushort.MaxValue);
            ushort minusField = UShortSafeDec(zeroZField, PediatricSoftConstants.SensorFieldStep, ushort.MinValue);

            double plusSum = 0;
            double minusSum = 0;

            SendCommand(PediatricSoftConstants.SensorCommandFieldZModulationAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultFieldZModulationAmplitude)));

            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataSelectorZDemod)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);

            for (int i = 0; i < PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps; i++)
            {
                SendCommand(String.Concat("#", UInt16ToStringBE(plusField)));
                lock (dataLock)
                {
                    dataUpdated = false;
                }
                while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                lock (dataLock)
                {
                    plusSum += (double)lastValueRAW;
                }

                SendCommand(String.Concat("#", UInt16ToStringBE(minusField)));
                lock (dataLock)
                {
                    dataUpdated = false;
                }
                while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                lock (dataLock)
                {
                    minusSum += (double)lastValueRAW;
                }

            }

            SendCommand(String.Concat("#", UInt16ToStringBE(zeroZField)));
            sensorZDemodCalibration = PediatricSoftConstants.SensorCoilsCalibrationTeslaPerHex / ((plusSum - minusSum) / PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps / 2 / PediatricSoftConstants.SensorFieldStep);

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response delta sum {(plusSum - minusSum)}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Number of averages {PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response delta {(plusSum - minusSum) / PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Number of hex steps {2 * PediatricSoftConstants.SensorFieldStep}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response per hex step {(plusSum - minusSum) / PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps / 2 / PediatricSoftConstants.SensorFieldStep}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: ZDemod calibration {sensorZDemodCalibration}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Calibration done");
        }

        private Task StateHandlerAsync()
        {
            return Task.Run(() =>
            {
                PediatricSoftConstants.SensorState currentState;

                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: State handler: started");

                lock (stateLock)
                {
                    currentState = State;
                }

                while (currentState < PediatricSoftConstants.SensorState.ShutDownComplete)
                {
                    switch (currentState)
                    {
                        case PediatricSoftConstants.SensorState.Setup:
                            SendCommandsSetup();
                            break;

                        case PediatricSoftConstants.SensorState.LaserLockSweep:
                            SendCommandsLaserLockSweep();
                            break;

                        case PediatricSoftConstants.SensorState.LaserLockStep:
                            SendCommandsLaserLockStep();
                            break;

                        case PediatricSoftConstants.SensorState.LaserLockPID:
                            SendCommandsLaserLockPID();
                            break;

                        case PediatricSoftConstants.SensorState.StabilizeCellHeat:
                            SendCommandsStabilizeCellHeat();
                            break;

                        case PediatricSoftConstants.SensorState.ZeroFields:
                            //SendCommandsZeroFields();
                            lock (stateLock)
                            {
                                State = PediatricSoftConstants.SensorState.Idle;
                            }
                            break;

                        case PediatricSoftConstants.SensorState.CalibrateMagnetometer:
                            //SendCommandsCalibrateMagnetometer();
                            break;

                        case PediatricSoftConstants.SensorState.ShutDownRequested:
                            if (!PediatricSensorData.DebugMode)
                            {
                                SendCommandsSetup();

                                // Disable streaming
                                SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndADCGain);
                                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataStreamingOffGainLow)));

                                while (commandQueue.TryPeek(out string dummy)) Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                            }
                            lock (stateLock)
                            {
                                State++;
                            }
                            break;

                        default:
                            Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                            break;
                    }

                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }

                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: State handler: exiting");

            });
        }

        public void SendCommand(string command)
        {
            command = Regex.Replace(command, @"[^\w@#?+-]", "");

            if (!String.IsNullOrEmpty(command))
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Sending command \"{command}\"");
                commandQueue.Enqueue(command + "\n");
            }
        }

        private void SaveConfig()
        {
            if (PediatricSensorConfig.Equals(PediatricSensorConfigOnLoad))
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Configuration not changed - not saving");
            else
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Saving configuration");

                TextWriter writer = null;
                try
                {
                    XmlSerializer serializer = new XmlSerializer(typeof(PediatricSensorConfig));
                    writer = new StreamWriter(configPath);
                    serializer.Serialize(writer, PediatricSensorConfig);
                }
                finally
                {
                    if (writer != null)
                        writer.Close();
                }

            }
        }

        private void LoadConfig()
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Loading configuration");

            TextReader reader = null;
            try
            {
                XmlSerializer serializer = new XmlSerializer(typeof(PediatricSensorConfig));
                reader = new StreamReader(configPath);
                PediatricSensorConfig = (PediatricSensorConfig)serializer.Deserialize(reader);
            }
            catch (Exception)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Failed to load configuration - using defaults");
                PediatricSensorConfig = new PediatricSensorConfig();
            }
            finally
            {
                if (reader != null)
                    reader.Close();
            }

            PediatricSensorConfigOnLoad = PediatricSensorConfig.GetValueCopy();
        }



        public void Dispose()
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Dispose() was called");

            SaveConfig();

            lock (stateLock)
            {
                if (State > PediatricSoftConstants.SensorState.Init && State < PediatricSoftConstants.SensorState.ShutDownRequested)
                {
                    State = PediatricSoftConstants.SensorState.ShutDownRequested;
                }
            }

            if (uiUpdateTimer != null)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Disposing the UI timer");
                uiUpdateTimer.Enabled = false;
                uiUpdateTimer.Elapsed -= OnUIUpdateTimerEvent;
                uiUpdateTimer.Dispose();
                uiUpdateTimer = null;
            }

            if (streamingTask != null)
            {
                streamingTask.Wait();
                streamingTask.Dispose();
            };

            if (stateHandlerTask != null)
            {
                stateHandlerTask.Wait();
                stateHandlerTask.Dispose();
            };

            if (_FTDI != null)
            {
                try { _FTDI.Close(); } catch { } finally { _FTDI = null; };
            }

            IsDisposed = true;
        }

        ~PediatricSensor()
        {
            if (!IsDisposed) Dispose();
        }

    }
}