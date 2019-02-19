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
using System.IO;
using System.Diagnostics;
using System.Timers;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using Prism.Commands;
using FTD2XX_NET;
using Prism.Mvvm;
using System.Xml.Serialization;

namespace PediatricSoft
{
    public sealed class PediatricSensor : BindableBase, IDisposable
    {

        private PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private PediatricSensorConfig PediatricSensorConfig;
        private PediatricSensorConfig PediatricSensorConfigOnLoad;
        private readonly string configPath;

        private FTDI _FTDIPort;
        private Task streamingTask;
        private Task processingTask;
        private Task dataSaveTask;
        private Task stateTask;
        private System.Timers.Timer uiUpdateTimer;
        //private Random rnd = new Random();
        private CancellationTokenSource stateHandlerCancellationTokenSource;
        private CancellationTokenSource streamCancellationTokenSource;
        private CancellationTokenSource fileSaveCancellationTokenSource;

        private readonly Object stateLock = new Object();
        private PediatricSoftConstants.SensorState state = PediatricSoftConstants.SensorState.Init;
        public PediatricSoftConstants.SensorState State
        {
            get { return state; }
            private set
            {
                state = value;
                RaisePropertyChanged();
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
            }
        }
        //public string StateDescription { get { return } }

        private int sensorADCColdValueRaw;
        private double sensorZDemodCalibration = 1;
        private ushort laserHeat = PediatricSoftConstants.SensorMinLaserHeat;
        private ushort zeroXField = PediatricSoftConstants.SensorColdFieldXOffset;
        private ushort zeroYField = PediatricSoftConstants.SensorColdFieldYOffset;
        private ushort zeroZField = PediatricSoftConstants.SensorColdFieldZOffset;

        private readonly byte[] buffer = new byte[PediatricSoftConstants.ProcessingBufferSize];
        private UInt32 bufferIndex = 0;
        private readonly Object bufferLock = new Object();

        private bool dataSaveEnable = false;
        private bool dataSaveRAW = true;
        private List<string> dataSaveBuffer = new List<string>();
        private readonly Object dataSaveLock = new Object();

        private bool infoRequested = false;
        private string requestedInfoString = string.Empty;

        private Queue<int> dataTimeRaw = new Queue<int>(PediatricSoftConstants.DataQueueLength);
        private Queue<int> dataValueRaw = new Queue<int>(PediatricSoftConstants.DataQueueLength);
        private Queue<int> dataValueRawRunningAvg = new Queue<int>(PediatricSoftConstants.DataQueueRunningAvgLength);
        private readonly Object dataLock = new Object();

        public ChartValues<ObservableValue> ChartValues = new ChartValues<ObservableValue>();

        private bool isPlotted = false;
        public bool IsPlotted
        {
            get { return isPlotted; }
            set { isPlotted = value; RaisePropertyChanged(); PediatricSensorData.UpdateSeriesCollection(); }
        }

        public int LastValue { get; private set; } = 0;
        public int LastTime { get; private set; } = 0;
        public double RunningAvg { get; private set; } = 0;
        private bool wasDataUpdated = false;
        public double Voltage
        {
            get
            {
                if (State < PediatricSoftConstants.SensorState.Start)
                    return RunningAvg * PediatricSoftConstants.SensorADCRawToVolts;
                else
                    return RunningAvg * sensorZDemodCalibration;
            }
        }

        public string Port { get; private set; } = String.Empty;
        public string IDN { get; private set; } = String.Empty;
        public string SN { get; private set; } = String.Empty;
        public string PortSN { get { return String.Concat(Port, " - ", SN); } }
        public bool IsDisposed { get; private set; } = false;
        public bool IsLocked { get; private set; } = false;
        public bool IsRunning { get; private set; } = false;
        public bool IsValid
        {
            get
            {
                if (State > PediatricSoftConstants.SensorState.Init && State < PediatricSoftConstants.SensorState.Failed) return true;
                else return false;
            }
        }


        private void OnUIUpdateTimerEvent(Object source, ElapsedEventArgs e)
        {
            RaisePropertyChanged("LastValue");
            RaisePropertyChanged("Voltage");
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

        private bool FTDIPurgeBuffers()
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            ftStatus = _FTDIPort.Purge(FTDI.FT_PURGE.FT_PURGE_RX + FTDI.FT_PURGE.FT_PURGE_TX);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to purge buffers on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                Dispose();
                return false;
            }
            else return true;
        }

        private bool FTDICheckRxBuffer()
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            UInt32 numBytes = 0;

            ftStatus = _FTDIPort.GetRxBytesAvailable(ref numBytes);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                Dispose();
                return false;
            }
            else return (numBytes > 0);
        }

        private UInt32 FTDIWriteString(string data)
        {
            // Write string data to the device

            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;
            UInt32 numBytesWritten = 0;

            // Note that the Write method is overloaded, so can write string or byte array data
            ftStatus = _FTDIPort.Write(data, data.Length, ref numBytesWritten);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to write to FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                Dispose();
                return numBytesWritten;
            }
            else return numBytesWritten;
        }

        private PediatricSensor() { }
        public PediatricSensor(string serial)
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            _FTDIPort = new FTDI();

            SN = serial;

            // Open the device by serial number
            ftStatus = _FTDIPort.OpenBySerialNumber(serial);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to open FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set Baud rate
            ftStatus = _FTDIPort.SetBaudRate(PediatricSoftConstants.SerialPortBaudRate);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set Baud rate on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set data characteristics - Data bits, Stop bits, Parity
            ftStatus = _FTDIPort.SetDataCharacteristics(FTDI.FT_DATA_BITS.FT_BITS_8, FTDI.FT_STOP_BITS.FT_STOP_BITS_1, FTDI.FT_PARITY.FT_PARITY_NONE);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set data characteristics on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set flow control - set RTS/CTS flow control - we don't have flow control
            ftStatus = _FTDIPort.SetFlowControl(FTDI.FT_FLOW_CONTROL.FT_FLOW_NONE, 0x11, 0x13);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set flow control on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set timeouts
            ftStatus = _FTDIPort.SetTimeouts(PediatricSoftConstants.SerialPortReadTimeout, PediatricSoftConstants.SerialPortWriteTimeout);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to set timeouts on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Get COM Port
            ftStatus = _FTDIPort.GetCOMPort(out string port);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to get the COM Port Name on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Update the Port property
            Port = port;

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Opened Port {Port} on FTDI device with S/N {serial}");

            bool success = false;

            success = FTDIPurgeBuffers();
            if (!success) return;

            UInt32 numBytes = FTDIWriteString("?\n");
            if (numBytes == 0)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Not valid");
                Dispose();
                return;
            }

            Thread.Sleep(PediatricSoftConstants.SerialPortSleepTime);

            success = FTDICheckRxBuffer();
            if (success)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Valid;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
            }
            else
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Not valid");
                Dispose();
            }

            FTDIPurgeBuffers();

            configPath = System.IO.Path.Combine(PediatricSensorData.Instance.SensorConfigFolderAbsolute, SN) + ".xml";
            LoadConfig();

        }

        public void KickOffTasks()
        {

            streamCancellationTokenSource = new CancellationTokenSource();
            streamingTask = Task.Run(() => StreamData(streamCancellationTokenSource.Token));
            processingTask = Task.Run(() => ProcessData());

            stateHandlerCancellationTokenSource = new CancellationTokenSource();
            stateTask = Task.Run(() => StateHandler(stateHandlerCancellationTokenSource.Token));

            uiUpdateTimer = new System.Timers.Timer(PediatricSoftConstants.UIUpdateInterval);
            uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
            uiUpdateTimer.Enabled = true;

        }

        public void Lock()
        {
            PediatricSoftConstants.SensorState currentState;

            lock (stateLock)
            {
                if (State == PediatricSoftConstants.SensorState.Valid)
                {
                    State++;
                    if (SN == "16")
                        State = PediatricSoftConstants.SensorState.Idle;
                }
                currentState = State;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");

            while (currentState != PediatricSoftConstants.SensorState.Idle)
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

            lock (stateLock)
            {
                if (PediatricSensorData.DebugMode && State == PediatricSoftConstants.SensorState.Valid)
                    State = PediatricSoftConstants.SensorState.Idle;

                if (State == PediatricSoftConstants.SensorState.Idle)
                    State++;

                currentState = State;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");

            while (currentState != PediatricSoftConstants.SensorState.Run)
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
                if (State == PediatricSoftConstants.SensorState.Run)
                    State++;

                currentState = State;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");

            while (currentState != PediatricSoftConstants.SensorState.Idle)
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

            lock (stateLock)
            {
                if (State == PediatricSoftConstants.SensorState.Idle)
                {
                    State = PediatricSoftConstants.SensorState.ZeroFields;
                    if (SN == "16")
                        State = PediatricSoftConstants.SensorState.Idle;
                }
                currentState = State;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");

            while (currentState != PediatricSoftConstants.SensorState.Idle)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }
        }

        private void StreamData(CancellationToken cancellationToken)
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            byte[] byteArrayIn = new byte[PediatricSoftConstants.SerialPortStreamBlockSize];
            UInt32 bytesRead = 0;

            while (!cancellationToken.IsCancellationRequested)
            {
                try
                {
                    // Note that the Read method is overloaded, so can read string or byte array data
                    // This thing blocks! Adjust the block size accordingly or call 
                    ftStatus = _FTDIPort.Read(byteArrayIn, PediatricSoftConstants.SerialPortStreamBlockSize, ref bytesRead);
                    if (ftStatus != FTDI.FT_STATUS.FT_OK)
                    {
                        lock (stateLock)
                        {
                            State = PediatricSoftConstants.SensorState.Failed;
                        }
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Failed to read in the Rx buffer on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                        return;
                    }
                }
                catch (TimeoutException)
                {

                }
                catch (Exception e)
                {
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Port {Port}: {e.Message}");
                    streamCancellationTokenSource.Cancel();
                    State = PediatricSoftConstants.SensorState.Failed;
                    RaisePropertyChanged("State");
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                }

                if (bytesRead > 0)
                {
                    lock (bufferLock)
                    {
                        Array.Copy(byteArrayIn, 0, buffer, bufferIndex, bytesRead);
                        bufferIndex += bytesRead;
                    }
                    bytesRead = 0;

                }
                //Thread.Sleep(rnd.Next(PediatricSoftConstants.SerialPortStreamSleepMin, PediatricSensorData.SerialPortStreamSleepMax));
            }

        }

        private void ProcessData()
        {
            byte[] data = new byte[PediatricSoftConstants.DataBlockSize];
            UInt32 dataIndex = 0;

            byte[] info = new byte[PediatricSoftConstants.InfoBlockSize];
            UInt32 infoIndex = 0;

            byte[] localBuffer = new byte[PediatricSoftConstants.ProcessingBufferSize];
            UInt32 localBufferIndex = 0;

            bool inEscape = false;
            bool inInfoFrame = false;

            int plotCounter = 0;
            const int plotCounterMax = PediatricSoftConstants.DataQueueLength / PediatricSoftConstants.PlotQueueLength;

            if (streamingTask != null)
            {
                while (streamingTask.Status == TaskStatus.Running)
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

                            case PediatricSoftConstants.FrameEscapeByte:
                                if (inEscape)
                                {
                                    data[dataIndex] = localBuffer[i];
                                    dataIndex++;
                                    inEscape = false;
                                }
                                else inEscape = true;
                                break;

                            case PediatricSoftConstants.StartDataFrameByte:
                                if (inEscape)
                                {
                                    data[dataIndex] = localBuffer[i];
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
                                    data[dataIndex] = localBuffer[i];
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
                                    info[infoIndex] = localBuffer[i];
                                    infoIndex++;
                                }
                                else
                                {
                                    data[dataIndex] = localBuffer[i];
                                    dataIndex++;
                                }
                                break;
                        }

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

                        if (dataIndex == PediatricSoftConstants.DataBlockSize)
                        {
                            lock (dataLock)
                            {
                                Array.Reverse(data); // switch from MSB to LSB
                                LastTime = BitConverter.ToInt32(data, 4);
                                LastValue = BitConverter.ToInt32(data, 0);

                                dataTimeRaw.Enqueue(LastTime);
                                if (dataTimeRaw.Count > PediatricSoftConstants.DataQueueLength) dataTimeRaw.Dequeue();

                                dataValueRaw.Enqueue(LastValue);
                                if (dataValueRaw.Count > PediatricSoftConstants.DataQueueLength) dataValueRaw.Dequeue();

                                dataValueRawRunningAvg.Enqueue(LastValue);
                                if (dataValueRawRunningAvg.Count > PediatricSoftConstants.DataQueueRunningAvgLength) dataValueRawRunningAvg.Dequeue();

                                RunningAvg = dataValueRawRunningAvg.Average();

                                lock (dataSaveLock)
                                {
                                    if (dataSaveEnable)
                                        if (dataSaveRAW)
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(LastTime), "\t", Convert.ToString(LastValue)));
                                        else
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(LastTime * 0.001), "\t", Convert.ToString(LastValue * sensorZDemodCalibration)));
                                }

                                if (!wasDataUpdated) wasDataUpdated = true;
                            }

                            if (IsPlotted)
                            {
                                plotCounter++;
                                if (plotCounter == plotCounterMax)
                                {
                                    if (State < PediatricSoftConstants.SensorState.Start)
                                        ChartValues.Add(new ObservableValue(RunningAvg * PediatricSoftConstants.SensorADCRawToVolts));
                                    else
                                        ChartValues.Add(new ObservableValue(RunningAvg * sensorZDemodCalibration));
                                    if (ChartValues.Count > PediatricSoftConstants.PlotQueueLength) ChartValues.RemoveAt(0);
                                    plotCounter = 0;
                                }
                            }

                            dataIndex = 0;
                        }

                    }

                    Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                };
            };
        }

        private void SaveData(CancellationToken token)
        {

            string filePath = System.IO.Path.Combine(PediatricSensorData.SaveFolderCurrentRun, SN);
            filePath += ".txt";

            while (!token.IsCancellationRequested)
            {
                Thread.Sleep(1000);
                lock (dataSaveLock)
                {
                    File.AppendAllLines(filePath, dataSaveBuffer);
                    dataSaveBuffer.Clear();
                }
            }

        }

        private void SendCommandsColdSensor()
        {
            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataSelectorADC)));

            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataStreamingOnGainLow)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockDisable)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrentMod);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserCurrentModValue)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdLaserCurrent)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdLaserHeat)));

            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdCellHeat)));

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

        }

        private void SendCommandsLaserLockSweep()
        {

            bool foundResonanceSweep = false;
            int numberOfLaserLockSweepCycles = 0;
            int[] data = new int[PediatricSoftConstants.DataQueueLength];
            double transmission = 0;

            // Turn on the laser
            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserCurrent)));

            // Wait a bit and record the ADC value
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Waiting for {PediatricSoftConstants.StateHandlerADCColdDelay} ms");
            Thread.Sleep(PediatricSoftConstants.StateHandlerADCColdDelay);
            lock (dataLock)
            {
                sensorADCColdValueRaw = LastValue;
            }

            // Here we check if we received non-zero value.
            // If it is zero (default) - something is wrong and we fail.
            if (sensorADCColdValueRaw == 0)
            {
                SendCommandsColdSensor();
                State = PediatricSoftConstants.SensorState.Failed;
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
            }
            else
            {
                // Set the cell heater to the max value, wait for the cell to warm up
                SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMaxCellHeat)));
                Thread.Sleep(PediatricSoftConstants.StateHandlerCellHeatInitialTime);
            }

            // Turn on the Z-coil to increase absorption
            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockFieldZOffset)));

            // Laser heat sweep cycle. Here we look for the Rb resonance.
            while (!foundResonanceSweep && State == PediatricSoftConstants.SensorState.LaserLockSweep && numberOfLaserLockSweepCycles < PediatricSoftConstants.MaxNumberOfLaserLockSweepCycles)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Laser lock sweep cycle: {numberOfLaserLockSweepCycles}");

                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Waiting for {PediatricSoftConstants.StateHandlerCellHeatInitialTime} ms");
                Thread.Sleep(PediatricSoftConstants.StateHandlerLaserHeatSweepTime);

                // Set the laser heater to the max value, wait a bit
                SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMaxLaserHeat)));
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
                    foundResonanceSweep = true;
                    SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                    SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMinLaserHeat)));

                }
                else if (numberOfLaserLockSweepCycles < PediatricSoftConstants.MaxNumberOfLaserLockSweepCycles)
                {
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, going to wait more");
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Minimal transmission {transmission} is higher than the target {PediatricSoftConstants.SensorTargetLaserTransmissionSweep}");
                    SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                    SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMinLaserHeat)));
                    numberOfLaserLockSweepCycles++;
                }

            }

            if (!foundResonanceSweep && State == PediatricSoftConstants.SensorState.LaserLockSweep)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, giving up");
                SendCommandsColdSensor();
                State = PediatricSoftConstants.SensorState.Failed;
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
            }

        }

        private void SendCommandsLaserLockStep()
        {

            bool foundResonanceStep = false;
            int numberOfLaserLockStepCycles = 0;
            double transmission = double.MaxValue;

            Thread.Sleep(PediatricSoftConstants.SensorLaserHeatStepCycleDelay);

            while (!foundResonanceStep && State == PediatricSoftConstants.SensorState.LaserLockStep && numberOfLaserLockStepCycles < PediatricSoftConstants.MaxNumberOfLaserLockStepCycles)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Laser lock step cycle: {numberOfLaserLockStepCycles}");

                laserHeat = PediatricSoftConstants.SensorMinLaserHeat;
                SendCommand(PediatricSoftConstants.SensorCommandLaserHeat, PediatricSoftConstants.IsLaserLockDebugEnabled);
                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)), PediatricSoftConstants.IsLaserLockDebugEnabled);

                while (!foundResonanceStep && State == PediatricSoftConstants.SensorState.LaserLockStep && laserHeat < PediatricSoftConstants.SensorMaxLaserHeat)
                {
                    if (transmission < PediatricSoftConstants.SensorTargetLaserTransmissionStep)
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Step cycle - found Rb resonance");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Transmission on resonance: {transmission}");
                        foundResonanceStep = true;
                    }
                    else
                    {
                        laserHeat = UShortSafeInc(laserHeat, PediatricSoftConstants.SensorLaserHeatStep, PediatricSoftConstants.SensorMaxLaserHeat);
                        SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            transmission = (double)LastValue / sensorADCColdValueRaw;
                        }
                    }
                }

                numberOfLaserLockStepCycles++;
            }

            if (!foundResonanceStep)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: We saw Rb resonance, but the step cycle failed. Giving up");
                SendCommandsColdSensor();
                State = PediatricSoftConstants.SensorState.Failed;
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
            }


        }

        private void SendCommandsLaserLockPID()
        {
            //SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndGain);
            //SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataStreamingOnGainHigh)));

            //sensorADCColdValueRaw = sensorADCColdValueRaw * 50 / 15;

            SendCommand(PediatricSoftConstants.SensorCommandLaserLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockEnable)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserHeat)));
        }

        private void SendCommandsStabilizeCellHeat()
        {
            double transmission = 0;

            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdCellHeat)));

            while (transmission < PediatricSoftConstants.SensorTargetLaserTransmissionSweep)
            {
                Thread.Sleep(PediatricSoftConstants.SensorLaserHeatStepSleepTime);
                lock (dataLock)
                {
                    transmission = (double)LastValue / sensorADCColdValueRaw;
                }
            }

            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultCellHeat)));
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
                        SendCommand(PediatricSoftConstants.SensorCommandFieldXOffset, PediatricSoftConstants.IsLaserLockDebugEnabled);
                        plusField = UShortSafeInc(zeroXField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroXField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            plusValue = (double)LastValue;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            minusValue = (double)LastValue;
                        }

                        if (plusValue > minusValue)
                            zeroXField = UShortSafeInc(zeroXField, PediatricSoftConstants.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroXField = UShortSafeDec(zeroXField, PediatricSoftConstants.SensorFieldStep, ushort.MinValue);

                        break;

                    case 1:
                        SendCommand(PediatricSoftConstants.SensorCommandFieldYOffset, PediatricSoftConstants.IsLaserLockDebugEnabled);
                        plusField = UShortSafeInc(zeroYField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroYField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            plusValue = (double)LastValue;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            minusValue = (double)LastValue;
                        }

                        if (plusValue > minusValue)
                            zeroYField = UShortSafeInc(zeroYField, PediatricSoftConstants.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroYField = UShortSafeDec(zeroYField, PediatricSoftConstants.SensorFieldStep, ushort.MinValue);

                        break;

                    case 2:
                        SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset, PediatricSoftConstants.IsLaserLockDebugEnabled);
                        plusField = UShortSafeInc(zeroZField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroZField, PediatricSoftConstants.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            plusValue = (double)LastValue;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            minusValue = (double)LastValue;
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

            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset, PediatricSoftConstants.IsLaserLockDebugEnabled);

            for (int i = 0; i < PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps; i++)
            {
                SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                lock (dataLock)
                {
                    wasDataUpdated = false;
                }
                while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                lock (dataLock)
                {
                    plusSum += (double)LastValue;
                }

                SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
                lock (dataLock)
                {
                    wasDataUpdated = false;
                }
                while (!wasDataUpdated) Thread.Sleep(PediatricSoftConstants.SerialPortStreamSleepMin);
                lock (dataLock)
                {
                    minusSum += (double)LastValue;
                }

            }

            SendCommand(String.Concat("#", UInt16ToStringBE(zeroZField)), PediatricSoftConstants.IsLaserLockDebugEnabled);
            sensorZDemodCalibration = PediatricSoftConstants.SensorCoilsCalibrationTeslaPerHex / ((plusSum - minusSum) / PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps / 2 / PediatricSoftConstants.SensorFieldStep);

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response delta sum {(plusSum - minusSum)}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Number of averages {PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response delta {(plusSum - minusSum) / PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Number of hex steps {2 * PediatricSoftConstants.SensorFieldStep}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response per hex step {(plusSum - minusSum) / PediatricSoftConstants.NumberOfMagnetometerCalibrationSteps / 2 / PediatricSoftConstants.SensorFieldStep}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: ZDemod calibration {sensorZDemodCalibration}");
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Calibration done");
        }

        private void StateHandler(CancellationToken stateHandlerCancellationToken)
        {

            while (!stateHandlerCancellationToken.IsCancellationRequested)
            {
                switch (State)
                {

                    case PediatricSoftConstants.SensorState.Valid:
                        break;

                    case PediatricSoftConstants.SensorState.MakeCold:

                        SendCommandsColdSensor();
                        if (State == PediatricSoftConstants.SensorState.MakeCold)
                        {
                            State++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        }
                        break;

                    case PediatricSoftConstants.SensorState.Cold:

                        State++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        break;

                    case PediatricSoftConstants.SensorState.LockStart:

                        if (State == PediatricSoftConstants.SensorState.LockStart)
                        {
                            State++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        }
                        break;

                    case PediatricSoftConstants.SensorState.LaserLockSweep:

                        SendCommandsLaserLockSweep();
                        if (State == PediatricSoftConstants.SensorState.LaserLockSweep)
                        {
                            State++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        }
                        break;

                    case PediatricSoftConstants.SensorState.LaserLockStep:

                        SendCommandsLaserLockStep();
                        if (State == PediatricSoftConstants.SensorState.LaserLockStep)
                        {
                            State++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        }
                        break;

                    case PediatricSoftConstants.SensorState.LaserLockPID:

                        SendCommandsLaserLockPID();
                        State++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        break;

                    case PediatricSoftConstants.SensorState.StabilizeCellHeat:

                        SendCommandsStabilizeCellHeat();
                        State++;
                        State = PediatricSoftConstants.SensorState.Idle;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        break;

                    case PediatricSoftConstants.SensorState.ZeroFields:

                        SendCommandsZeroFields();
                        State++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        break;

                    case PediatricSoftConstants.SensorState.CalibrateMagnetometer:

                        SendCommandsCalibrateMagnetometer();
                        State++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        break;

                    case PediatricSoftConstants.SensorState.Idle:
                        break;

                    case PediatricSoftConstants.SensorState.Start:

                        if (PediatricSensorData.SaveDataEnabled)
                        {
                            lock (dataSaveLock)
                            {
                                dataSaveEnable = true;
                                dataSaveRAW = PediatricSensorData.SaveRAWValues;
                            }
                            fileSaveCancellationTokenSource = new CancellationTokenSource();
                            dataSaveTask = Task.Run(() => SaveData(fileSaveCancellationTokenSource.Token));
                        }

                        State++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        break;

                    case PediatricSoftConstants.SensorState.Run:
                        break;

                    case PediatricSoftConstants.SensorState.Stop:

                        if (PediatricSensorData.SaveDataEnabled)
                        {
                            lock (dataSaveLock)
                            {
                                dataSaveEnable = false;
                            }
                            fileSaveCancellationTokenSource.Cancel();
                            dataSaveTask.Wait();
                            fileSaveCancellationTokenSource.Dispose();
                            fileSaveCancellationTokenSource = null;
                            dataSaveTask.Dispose();
                            dataSaveTask = null;
                        }

                        if (State == PediatricSoftConstants.SensorState.Stop)
                        {
                            State = PediatricSoftConstants.SensorState.Idle;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {State}");
                        }
                        break;

                    case PediatricSoftConstants.SensorState.Failed:
                        break;

                    case PediatricSoftConstants.SensorState.ShutDown:
                        break;

                    default:
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Unknown state. We shouldn't be here.");
                        break;
                }

                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
            }

            if (!PediatricSensorData.DebugMode)
                SendCommandsColdSensor();

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: State Handler exiting.");
        }

        public void SendCommand(string command)
        {
            SendCommandMain(command, true);
        }

        public void SendCommand(string command, bool debug)
        {
            SendCommandMain(command, debug);
        }

        private void SendCommandMain(string command, bool debug)
        {

            command = Regex.Replace(command, @"[^\w@#?+-]", "");

            if (State > PediatricSoftConstants.SensorState.Init && State < PediatricSoftConstants.SensorState.ShutDown)
            {
                if (!String.IsNullOrEmpty(command))
                {
                    try
                    {
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled && debug, $"Sensor {SN} on port {Port}: sending command \"{command}\"");
                        FTDIWriteString($"{command}\n");
                    }
                    catch (Exception e) { Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, e.Message); }


                }
            }
            else
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Can't send commands - sensor not running");

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
                if (State > PediatricSoftConstants.SensorState.Init)
                {
                    //state = PediatricSoftConstants.SensorState.ShutDown;
                }
            }

            if (stateTask != null)
            {
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Cancelling the state handler task");
                stateHandlerCancellationTokenSource.Cancel();
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Waiting for the state handler task to finish");
                stateTask.Wait();
                stateTask.Dispose();
                stateTask = null;
                stateHandlerCancellationTokenSource.Dispose();
                stateHandlerCancellationTokenSource = null;
                Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: State handler cleanup done");

                if (uiUpdateTimer != null)
                {
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Sensor {SN} on port {Port}: Disposing the UI timer");
                    uiUpdateTimer.Enabled = false;
                    uiUpdateTimer.Elapsed -= OnUIUpdateTimerEvent;
                    uiUpdateTimer.Dispose();
                    uiUpdateTimer = null;
                }
            }

            lock (dataSaveLock)
            {
                dataSaveEnable = false;
            }
            if (fileSaveCancellationTokenSource != null)
            {
                try
                {
                    fileSaveCancellationTokenSource.Cancel();
                    dataSaveTask.Wait();
                    fileSaveCancellationTokenSource.Dispose();
                    fileSaveCancellationTokenSource = null;
                    dataSaveTask.Dispose();
                    dataSaveTask = null;
                }
                catch (Exception) { }
            }
            if (dataSaveTask != null)
            {
                try
                {
                    dataSaveTask.Wait();
                    dataSaveTask.Dispose();
                    dataSaveTask = null;
                }
                catch (Exception) { }
            }

            if (streamCancellationTokenSource != null)
                try
                {
                    streamCancellationTokenSource.Cancel();
                }
                catch (Exception) { }


            if (streamingTask != null)
            {
                streamingTask.Wait();
                streamingTask.Dispose();
            };

            if (processingTask != null)
            {
                processingTask.Wait();
                processingTask.Dispose();
            };

            if (streamCancellationTokenSource != null)
                try
                {
                    streamCancellationTokenSource.Dispose();
                }
                catch (Exception) { }

            if (stateTask != null)
            {
                stateTask.Wait();
                stateTask.Dispose();
            };

            if (_FTDIPort != null)
            {
                try { _FTDIPort.Close(); } catch { } finally { _FTDIPort = null; };
            }

            IsDisposed = true;
        }

        ~PediatricSensor()
        {
            if (!IsDisposed) Dispose();
        }

    }
}