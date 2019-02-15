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

namespace PediatricSoft
{
    public sealed class PediatricSensor : BindableBase, IDisposable
    {

        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

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

        private byte state = PediatricSensorData.SensorStateInit;
        private readonly Object stateLock = new Object();

        private int sensorADCColdValueRaw;
        private double sensorZDemodCalibration = 1;
        private ushort laserHeat = PediatricSensorData.SensorMinLaserHeat;
        private ushort zeroXField = PediatricSensorData.SensorColdFieldXOffset;
        private ushort zeroYField = PediatricSensorData.SensorColdFieldYOffset;
        private ushort zeroZField = PediatricSensorData.SensorColdFieldZOffset;

        private string optimalLaserCurrentString = string.Empty;
        private string optimalBzModString = string.Empty;
        private string optimalCellHeatLockPointString = string.Empty;
        private string optimalMaxCellHeatString = string.Empty;

        private readonly byte[] buffer = new byte[PediatricSensorData.ProcessingBufferSize];
        private UInt32 bufferIndex = 0;
        private readonly Object bufferLock = new Object();

        private bool dataSaveEnable = false;
        private bool dataSaveRAW = true;
        private List<string> dataSaveBuffer = new List<string>();
        private readonly Object dataSaveLock = new Object();

        private bool infoRequested = false;
        private string requestedInfoString = string.Empty;

        private Queue<int> dataTimeRaw = new Queue<int>(PediatricSensorData.DataQueueLength);
        private Queue<int> dataValueRaw = new Queue<int>(PediatricSensorData.DataQueueLength);
        private Queue<int> dataValueRawRunningAvg = new Queue<int>(PediatricSensorData.DataQueueRunningAvgLength);
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
                if (state < PediatricSensorData.SensorStateStart)
                    return RunningAvg * PediatricSensorData.SensorADCRawToVolts;
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
                if (state > PediatricSensorData.SensorStateInit && state < PediatricSensorData.SensorStateFailed) return true;
                else return false;
            }
        }
        public byte State { get { return state; } }

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
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to purge buffers on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
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
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
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
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to write to FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
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
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to open FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set Baud rate
            ftStatus = _FTDIPort.SetBaudRate(PediatricSensorData.SerialPortBaudRate);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to set Baud rate on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set data characteristics - Data bits, Stop bits, Parity
            ftStatus = _FTDIPort.SetDataCharacteristics(FTDI.FT_DATA_BITS.FT_BITS_8, FTDI.FT_STOP_BITS.FT_STOP_BITS_1, FTDI.FT_PARITY.FT_PARITY_NONE);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to set data characteristics on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set flow control - set RTS/CTS flow control - we don't have flow control
            ftStatus = _FTDIPort.SetFlowControl(FTDI.FT_FLOW_CONTROL.FT_FLOW_NONE, 0x11, 0x13);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to set flow control on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Set timeouts
            ftStatus = _FTDIPort.SetTimeouts(PediatricSensorData.SerialPortReadTimeout, PediatricSensorData.SerialPortWriteTimeout);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to set timeouts on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Get COM Port
            ftStatus = _FTDIPort.GetCOMPort(out string port);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                lock (stateLock)
                {
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to get the COM Port Name on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                Dispose();
                return;
            }

            // Update the Port property
            Port = port;

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Opened Port {Port} on FTDI device with S/N {serial}");

            bool success = false;

            success = FTDIPurgeBuffers();
            if (!success) return;

            UInt32 numBytes = FTDIWriteString("?\n");
            if (numBytes == 0)
            {
                lock (stateLock)
                {
                    state = PediatricSensorData.SensorStateFailed;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Not valid");
                Dispose();
                return;
            }

            Thread.Sleep(PediatricSensorData.SerialPortSleepTime);

            success = FTDICheckRxBuffer();
            if (success)
            {
                lock (stateLock)
                {
                    state = PediatricSensorData.SensorStateValid;
                }
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }
            else
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Not valid");
                Dispose();
            }

            FTDIPurgeBuffers();

        }

        public void KickOffTasks()
        {

            streamCancellationTokenSource = new CancellationTokenSource();
            streamingTask = Task.Run(() => StreamData(streamCancellationTokenSource.Token));
            processingTask = Task.Run(() => ProcessData());

            stateHandlerCancellationTokenSource = new CancellationTokenSource();
            stateTask = Task.Run(() => StateHandler(stateHandlerCancellationTokenSource.Token));

            uiUpdateTimer = new System.Timers.Timer(PediatricSensorData.UIUpdateInterval);
            uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
            uiUpdateTimer.Enabled = true;

        }

        public void Lock()
        {
            byte currentState;

            lock (stateLock)
            {
                if (state == PediatricSensorData.SensorStateValid)
                {
                    state++;
                    if (SN == "16")
                        state = PediatricSensorData.SensorStateIdle;
                }
                currentState = state;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

            while (currentState != PediatricSensorData.SensorStateIdle)
            {
                Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = state;
                }
            }
        }

        public void Start()
        {
            byte currentState;

            lock (stateLock)
            {
                if (PediatricSensorData.DebugMode && state == PediatricSensorData.SensorStateValid)
                    state = PediatricSensorData.SensorStateIdle;

                if (state == PediatricSensorData.SensorStateIdle)
                    state++;

                currentState = state;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

            while (currentState != PediatricSensorData.SensorStateRun)
            {
                Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = state;
                }
            }
        }

        public void Stop()
        {
            byte currentState;

            lock (stateLock)
            {
                if (state == PediatricSensorData.SensorStateRun)
                    state++;

                currentState = state;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

            while (currentState != PediatricSensorData.SensorStateIdle)
            {
                Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = state;
                }
            }
        }

        public void ZeroFields()
        {
            byte currentState;

            lock (stateLock)
            {
                if (state == PediatricSensorData.SensorStateIdle)
                {
                    state = PediatricSensorData.SensorStateZeroFields;
                    if (SN == "16")
                        state = PediatricSensorData.SensorStateIdle;
                }
                currentState = state;
            }

            RaisePropertyChanged("State");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

            while (currentState != PediatricSensorData.SensorStateIdle)
            {
                Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = state;
                }
            }
        }

        private void StreamData(CancellationToken cancellationToken)
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            byte[] byteArrayIn = new byte[PediatricSensorData.SerialPortStreamBlockSize];
            UInt32 bytesRead = 0;

            while (!cancellationToken.IsCancellationRequested)
            {
                try
                {
                    // Note that the Read method is overloaded, so can read string or byte array data
                    // This thing blocks! Adjust the block size accordingly or call 
                    ftStatus = _FTDIPort.Read(byteArrayIn, PediatricSensorData.SerialPortStreamBlockSize, ref bytesRead);
                    if (ftStatus != FTDI.FT_STATUS.FT_OK)
                    {
                        lock (stateLock)
                        {
                            state = PediatricSensorData.SensorStateFailed;
                        }
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Failed to read in the Rx buffer on FTDI device with S/N {SN} on Port {Port}. Error: {ftStatus.ToString()}");
                        return;
                    }
                }
                catch (TimeoutException)
                {

                }
                catch (Exception e)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: {e.Message}");
                    streamCancellationTokenSource.Cancel();
                    state = PediatricSensorData.SensorStateFailed;
                    RaisePropertyChanged("State");
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
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
                //Thread.Sleep(rnd.Next(PediatricSensorData.SerialPortStreamSleepMin, PediatricSensorData.SerialPortStreamSleepMax));
            }

        }

        private void ProcessData()
        {
            byte[] data = new byte[PediatricSensorData.DataBlockSize];
            UInt32 dataIndex = 0;

            byte[] info = new byte[PediatricSensorData.InfoBlockSize];
            UInt32 infoIndex = 0;

            byte[] localBuffer = new byte[PediatricSensorData.ProcessingBufferSize];
            UInt32 localBufferIndex = 0;

            bool inEscape = false;
            bool inInfoFrame = false;

            int plotCounter = 0;
            const int plotCounterMax = PediatricSensorData.DataQueueLength / PediatricSensorData.PlotQueueLength;

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
                                else
                                {
                                    inInfoFrame = false;
                                    infoIndex = 0;
                                    dataIndex = 0;
                                }
                                break;

                            case PediatricSensorData.StartInfoFrameByte:
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

                        if (infoIndex == PediatricSensorData.InfoBlockSize)
                        {
                            string infoMessage = System.Text.Encoding.ASCII.GetString(info);
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Info message: {infoMessage}");
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

                        if (dataIndex == PediatricSensorData.DataBlockSize)
                        {
                            lock (dataLock)
                            {
                                Array.Reverse(data); // switch from MSB to LSB
                                LastTime = BitConverter.ToInt32(data, 4);
                                LastValue = BitConverter.ToInt32(data, 0);

                                dataTimeRaw.Enqueue(LastTime);
                                if (dataTimeRaw.Count > PediatricSensorData.DataQueueLength) dataTimeRaw.Dequeue();

                                dataValueRaw.Enqueue(LastValue);
                                if (dataValueRaw.Count > PediatricSensorData.DataQueueLength) dataValueRaw.Dequeue();

                                dataValueRawRunningAvg.Enqueue(LastValue);
                                if (dataValueRawRunningAvg.Count > PediatricSensorData.DataQueueRunningAvgLength) dataValueRawRunningAvg.Dequeue();

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
                                    if (state < PediatricSensorData.SensorStateStart)
                                        ChartValues.Add(new ObservableValue(RunningAvg * PediatricSensorData.SensorADCRawToVolts));
                                    else
                                        ChartValues.Add(new ObservableValue(RunningAvg * sensorZDemodCalibration));
                                    if (ChartValues.Count > PediatricSensorData.PlotQueueLength) ChartValues.RemoveAt(0);
                                    plotCounter = 0;
                                }
                            }

                            dataIndex = 0;
                        }

                    }

                    Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
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

        private void SendCommandsGetOptimalParameters()
        {
            // Get optimal laser current
            lock (dataLock)
            {
                infoRequested = true;
            }
            SendCommand(PediatricSensorData.SensorCommandGetOptimalLaserCurrent);
            SendCommand("?");
            while (infoRequested) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Optimal Laser Current: {requestedInfoString}");
            optimalLaserCurrentString = requestedInfoString;

            // Get optimal Bz modulation amplitude
            lock (dataLock)
            {
                infoRequested = true;
            }
            SendCommand(PediatricSensorData.SensorCommandGetOptimalBzMod);
            SendCommand("?");
            while (infoRequested) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Optimal Bz modulation amplitude: {requestedInfoString}");
            optimalBzModString = requestedInfoString;

            // Get Optimal Cell Heat Lock Point
            lock (dataLock)
            {
                infoRequested = true;
            }
            SendCommand(PediatricSensorData.SensorCommandGetOptimalCellHeatLockPoint);
            SendCommand("?");
            while (infoRequested) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Optimal Cell Heat Lock Point: {requestedInfoString}");
            optimalCellHeatLockPointString = requestedInfoString;

            // Get Max Cell Heat
            lock (dataLock)
            {
                infoRequested = true;
            }
            SendCommand(PediatricSensorData.SensorCommandGetMaxCellHeat);
            SendCommand("?");
            while (infoRequested) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Max Cell Heat: {requestedInfoString}");
            optimalMaxCellHeatString = requestedInfoString;
        }

        private void SendCommandsColdSensor()
        {
            SendCommand(PediatricSensorData.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataSelectorADC)));

            SendCommand(PediatricSensorData.SensorCommandDigitalDataStreamingAndGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataStreamingOnGainLow)));

            SendCommand(PediatricSensorData.SensorCommandLaserlock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserlockDisable)));

            //SendCommand(PediatricSensorData.SensorCommandLaserCurrentMod);
            //SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserCurrentModValue)));

            SendCommand(PediatricSensorData.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdLaserCurrent)));

            SendCommand(PediatricSensorData.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdLaserHeat)));

            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdCellHeat)));

            SendCommand(PediatricSensorData.SensorCommandFieldXOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldXOffset)));

            SendCommand(PediatricSensorData.SensorCommandFieldXAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldXAmplitude)));

            SendCommand(PediatricSensorData.SensorCommandFieldYOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldYOffset)));

            SendCommand(PediatricSensorData.SensorCommandFieldYAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldYAmplitude)));

            SendCommand(PediatricSensorData.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldZOffset)));

            SendCommand(PediatricSensorData.SensorCommandFieldZAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldZAmplitude)));

        }

        private void SendCommandsLaserLockSweep()
        {

            bool foundResonanceSweep = false;
            int numberOfLaserLockSweepCycles = 0;
            int[] data = new int[PediatricSensorData.DataQueueLength];
            double transmission = 0;

            // Turn on the laser
            SendCommand(PediatricSensorData.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", optimalLaserCurrentString));

            // Wait a bit and record the ADC value
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerADCColdDelay} ms");
            Thread.Sleep(PediatricSensorData.StateHandlerADCColdDelay);
            lock (dataLock)
            {
                sensorADCColdValueRaw = LastValue;
            }

            // Here we check if we received non-zero value.
            // If it is zero (default) - something is wrong and we fail.
            if (sensorADCColdValueRaw == 0)
            {
                SendCommandsColdSensor();
                state = PediatricSensorData.SensorStateFailed;
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }
            else
            {
                // Set the cell heater to the max value, wait for the cell to warm up
                SendCommand(PediatricSensorData.SensorCommandCellHeat);
                SendCommand(String.Concat("#", optimalMaxCellHeatString));
                Thread.Sleep(PediatricSensorData.StateHandlerCellHeatInitialTime);
            }

            // Turn on the Z-coil to increase absorption
            SendCommand(PediatricSensorData.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserLockFieldZOffset)));

            // Laser heat sweep cycle. Here we look for the Rb resonance.
            while (!foundResonanceSweep && state == PediatricSensorData.SensorStateLaserLockSweep && numberOfLaserLockSweepCycles < PediatricSensorData.MaxNumberOfLaserLockSweepCycles)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Laser lock sweep cycle: {numberOfLaserLockSweepCycles}");

                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerCellHeatInitialTime} ms");
                Thread.Sleep(PediatricSensorData.StateHandlerLaserHeatSweepTime);

                // Set the laser heater to the max value, wait a bit
                SendCommand(PediatricSensorData.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorMaxLaserHeat)));
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerLaserHeatSweepTime} ms");
                Thread.Sleep(PediatricSensorData.StateHandlerLaserHeatSweepTime);

                // See if the threshold was reached
                lock (dataLock)
                {
                    data = dataValueRaw.ToArray();
                }
                transmission = (double)data.Min() / sensorADCColdValueRaw;
                if (transmission < PediatricSensorData.SensorTargetLaserTransmissionSweep)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Sweep cycle - found Rb resonance");
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Transmission on resonance: {transmission}");
                    foundResonanceSweep = true;
                    SendCommand(PediatricSensorData.SensorCommandLaserHeat);
                    SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorMinLaserHeat)));

                }
                else if (numberOfLaserLockSweepCycles < PediatricSensorData.MaxNumberOfLaserLockSweepCycles)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, going to wait more");
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Minimal transmission {transmission} is higher than the target {PediatricSensorData.SensorTargetLaserTransmissionSweep}");
                    SendCommand(PediatricSensorData.SensorCommandLaserHeat);
                    SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorMinLaserHeat)));
                    numberOfLaserLockSweepCycles++;
                }

            }

            if (!foundResonanceSweep && state == PediatricSensorData.SensorStateLaserLockSweep)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Didn't find Rb resonance, giving up");
                SendCommandsColdSensor();
                state = PediatricSensorData.SensorStateFailed;
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }

        }

        private void SendCommandsLaserLockStep()
        {

            bool foundResonanceStep = false;
            int numberOfLaserLockStepCycles = 0;
            double transmission = double.MaxValue;

            Thread.Sleep(PediatricSensorData.SensorLaserHeatStepCycleDelay);

            while (!foundResonanceStep && state == PediatricSensorData.SensorStateLaserLockStep && numberOfLaserLockStepCycles < PediatricSensorData.MaxNumberOfLaserLockStepCycles)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Laser lock step cycle: {numberOfLaserLockStepCycles}");

                laserHeat = PediatricSensorData.SensorMinLaserHeat;
                SendCommand(PediatricSensorData.SensorCommandLaserHeat, PediatricSensorData.IsLaserLockDebugEnabled);
                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)), PediatricSensorData.IsLaserLockDebugEnabled);

                while (!foundResonanceStep && state == PediatricSensorData.SensorStateLaserLockStep && laserHeat < PediatricSensorData.SensorMaxLaserHeat)
                {
                    if (transmission < PediatricSensorData.SensorTargetLaserTransmissionStep)
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Step cycle - found Rb resonance");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Transmission on resonance: {transmission}");
                        foundResonanceStep = true;
                    }
                    else
                    {
                        laserHeat = UShortSafeInc(laserHeat, PediatricSensorData.SensorLaserHeatStep, PediatricSensorData.SensorMaxLaserHeat);
                        SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)), PediatricSensorData.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
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
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: We saw Rb resonance, but the step cycle failed. Giving up");
                SendCommandsColdSensor();
                state = PediatricSensorData.SensorStateFailed;
                RaisePropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }


        }

        private void SendCommandsLaserLockPID()
        {
            //SendCommand(PediatricSensorData.SensorCommandDigitalDataStreamingAndGain);
            //SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataStreamingOnGainHigh)));

            //sensorADCColdValueRaw = sensorADCColdValueRaw * 50 / 15;

            SendCommand(PediatricSensorData.SensorCommandLaserlock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserlockEnable)));

            SendCommand(PediatricSensorData.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDefaultLaserHeat)));
        }

        private void SendCommandsStabilizeCellHeat()
        {
            double transmission = 0;

            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdCellHeat)));

            while (transmission < PediatricSensorData.SensorTargetLaserTransmissionSweep)
            {
                Thread.Sleep(PediatricSensorData.SensorLaserHeatStepSleepTime);
                lock (dataLock)
                {
                    transmission = (double)LastValue / sensorADCColdValueRaw;
                }
            }

            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorRunCellHeat)));
        }

        private void SendCommandsZeroFields()
        {
            int reminder = 0;
            ushort plusField = ushort.MaxValue;
            ushort minusField = ushort.MinValue;
            double plusValue = 0;
            double minusValue = 0;

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Begin field zeroing");

            SendCommand(PediatricSensorData.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataSelectorADC)));

            SendCommand(PediatricSensorData.SensorCommandFieldZAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldZAmplitude)));

            for (int i = 0; i < (PediatricSensorData.NumberOfFieldZeroingSteps * 3); i++)
            {
                Math.DivRem(i, 3, out reminder);
                switch (reminder)
                {
                    case -1:
                        SendCommand(PediatricSensorData.SensorCommandFieldXOffset, PediatricSensorData.IsLaserLockDebugEnabled);
                        plusField = UShortSafeInc(zeroXField, PediatricSensorData.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroXField, PediatricSensorData.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            plusValue = (double)LastValue;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            minusValue = (double)LastValue;
                        }

                        if (plusValue > minusValue)
                            zeroXField = UShortSafeInc(zeroXField, PediatricSensorData.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroXField = UShortSafeDec(zeroXField, PediatricSensorData.SensorFieldStep, ushort.MinValue);

                        break;

                    case 1:
                        SendCommand(PediatricSensorData.SensorCommandFieldYOffset, PediatricSensorData.IsLaserLockDebugEnabled);
                        plusField = UShortSafeInc(zeroYField, PediatricSensorData.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroYField, PediatricSensorData.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            plusValue = (double)LastValue;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            minusValue = (double)LastValue;
                        }

                        if (plusValue > minusValue)
                            zeroYField = UShortSafeInc(zeroYField, PediatricSensorData.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroYField = UShortSafeDec(zeroYField, PediatricSensorData.SensorFieldStep, ushort.MinValue);

                        break;

                    case 2:
                        SendCommand(PediatricSensorData.SensorCommandFieldZOffset, PediatricSensorData.IsLaserLockDebugEnabled);
                        plusField = UShortSafeInc(zeroZField, PediatricSensorData.SensorFieldCheckRange, ushort.MaxValue);
                        minusField = UShortSafeDec(zeroZField, PediatricSensorData.SensorFieldCheckRange, ushort.MinValue);

                        SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            plusValue = (double)LastValue;
                        }

                        SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                        lock (dataLock)
                        {
                            wasDataUpdated = false;
                        }
                        while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        lock (dataLock)
                        {
                            minusValue = (double)LastValue;
                        }

                        if (plusValue > minusValue)
                            zeroZField = UShortSafeInc(zeroZField, PediatricSensorData.SensorFieldStep, ushort.MaxValue);

                        if (plusValue < minusValue)
                            zeroZField = UShortSafeDec(zeroZField, PediatricSensorData.SensorFieldStep, ushort.MinValue);

                        break;

                    default:
                        break;
                }
            }

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Field zeroing done");
        }

        private void SendCommandsCalibrateMagnetometer()
        {
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Begin calibration");

            ushort plusField = UShortSafeInc(zeroZField, PediatricSensorData.SensorFieldStep, ushort.MaxValue);
            ushort minusField = UShortSafeDec(zeroZField, PediatricSensorData.SensorFieldStep, ushort.MinValue);

            double plusSum = 0;
            double minusSum = 0;

            SendCommand(PediatricSensorData.SensorCommandFieldZAmplitude);
            SendCommand(String.Concat("#", optimalBzModString));

            SendCommand(PediatricSensorData.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataSelectorZDemod)));

            SendCommand(PediatricSensorData.SensorCommandFieldZOffset, PediatricSensorData.IsLaserLockDebugEnabled);

            for (int i = 0; i < PediatricSensorData.NumberOfMagnetometerCalibrationSteps; i++)
            {
                SendCommand(String.Concat("#", UInt16ToStringBE(plusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                lock (dataLock)
                {
                    wasDataUpdated = false;
                }
                while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                lock (dataLock)
                {
                    plusSum += (double)LastValue;
                }

                SendCommand(String.Concat("#", UInt16ToStringBE(minusField)), PediatricSensorData.IsLaserLockDebugEnabled);
                lock (dataLock)
                {
                    wasDataUpdated = false;
                }
                while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                lock (dataLock)
                {
                    minusSum += (double)LastValue;
                }

            }

            SendCommand(String.Concat("#", UInt16ToStringBE(zeroZField)), PediatricSensorData.IsLaserLockDebugEnabled);
            sensorZDemodCalibration = PediatricSensorData.SensorCoilsCalibrationTeslaPerHex / ((plusSum - minusSum) / PediatricSensorData.NumberOfMagnetometerCalibrationSteps / 2 / PediatricSensorData.SensorFieldStep);

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response delta sum {(plusSum - minusSum)}");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Number of averages {PediatricSensorData.NumberOfMagnetometerCalibrationSteps}");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response delta {(plusSum - minusSum) / PediatricSensorData.NumberOfMagnetometerCalibrationSteps}");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Number of hex steps {2 * PediatricSensorData.SensorFieldStep}");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Response per hex step {(plusSum - minusSum) / PediatricSensorData.NumberOfMagnetometerCalibrationSteps / 2 / PediatricSensorData.SensorFieldStep}");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: ZDemod calibration {sensorZDemodCalibration}");
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Calibration done");
        }

        private void StateHandler(CancellationToken stateHandlerCancellationToken)
        {

            while (!stateHandlerCancellationToken.IsCancellationRequested)
            {
                switch (state)
                {

                    case PediatricSensorData.SensorStateValid:
                        break;

                    case PediatricSensorData.SensorStateGetOptimalParameters:

                        SendCommandsGetOptimalParameters();
                        if (state == PediatricSensorData.SensorStateGetOptimalParameters)
                        {
                            state++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateMakeCold:

                        SendCommandsColdSensor();
                        if (state == PediatricSensorData.SensorStateMakeCold)
                        {
                            state++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateCold:

                        state++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateLockStart:

                        if (state == PediatricSensorData.SensorStateLockStart)
                        {
                            state++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLockSweep:

                        SendCommandsLaserLockSweep();
                        if (state == PediatricSensorData.SensorStateLaserLockSweep)
                        {
                            state++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLockStep:

                        SendCommandsLaserLockStep();
                        if (state == PediatricSensorData.SensorStateLaserLockStep)
                        {
                            state++;
                            RaisePropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLockPID:

                        SendCommandsLaserLockPID();
                        state++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateStabilizeCellHeat:

                        SendCommandsStabilizeCellHeat();
                        state++;
                        state = PediatricSensorData.SensorStateIdle;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateZeroFields:

                        SendCommandsZeroFields();
                        state++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateCalibrateMagnetometer:

                        SendCommandsCalibrateMagnetometer();
                        state++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateIdle:
                        break;

                    case PediatricSensorData.SensorStateStart:

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

                        state++;
                        RaisePropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateRun:
                        break;

                    case PediatricSensorData.SensorStateStop:

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

                        if (state == PediatricSensorData.SensorStateStop)
                        {
                            state = PediatricSensorData.SensorStateIdle;
                            RaisePropertyChanged("State");
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

            if (!PediatricSensorData.DebugMode)
                SendCommandsColdSensor();

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: State Handler exiting.");
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

            if (state > PediatricSensorData.SensorStateInit && state < PediatricSensorData.SensorStateShutDown)
            {
                if (!String.IsNullOrEmpty(command))
                {
                    try
                    {
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled && debug, $"Sensor {SN} on port {Port}: sending command \"{command}\"");
                        FTDIWriteString($"{command}\n");
                    }
                    catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, e.Message); }


                }
            }
            else
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Can't send commands - sensor not running");

        }

        public void Dispose()
        {
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Dispose() was called");

            lock (stateLock)
            {
                if (state > PediatricSensorData.SensorStateInit)
                {
                    //state = PediatricSensorData.SensorStateShutDown;
                }
            }

            if (stateTask != null)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Cancelling the state handler task");
                stateHandlerCancellationTokenSource.Cancel();
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Waiting for the state handler task to finish");
                stateTask.Wait();
                stateTask.Dispose();
                stateTask = null;
                stateHandlerCancellationTokenSource.Dispose();
                stateHandlerCancellationTokenSource = null;
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: State handler cleanup done");

                if (uiUpdateTimer != null)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Disposing the UI timer");
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