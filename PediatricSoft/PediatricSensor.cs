using FTD2XX_NET;
using LiveCharts;
using Prism.Mvvm;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using System.Xml.Serialization;
using System.Numerics;
using MathNet.Numerics.IntegralTransforms;
using System.Diagnostics;

namespace PediatricSoft
{
    public sealed class PediatricSensor : BindableBase, IDisposable
    {
        // Fields
        private PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;
        private PediatricSensorConfig pediatricSensorConfigOnLoad;
        private string configPath;

        private FTDI _FTDI;
        private Task streamingTask;
        private Task stateHandlerTask;
        private System.Timers.Timer uiUpdateTimer;

        private readonly Object stateLock = new Object();
        private readonly Object dataLock = new Object();

        private double coldSensorADCValue = 0;
        private double CalibrationBzDemod = 0;
        private ushort zeroXField = PediatricSoftConstants.SensorColdFieldXOffset;
        private ushort zeroYField = PediatricSoftConstants.SensorColdFieldYOffset;
        private ushort zeroZField = PediatricSoftConstants.SensorColdFieldZOffset;

        private bool infoRequested = false;
        private string requestedInfoString = string.Empty;

        private ConcurrentQueue<string> commandQueue = new ConcurrentQueue<string>();

        private DataPoint lastDataPoint;

        private bool dataUpdated = false;

        private DataPoint[] dataPoints = new DataPoint[PediatricSoftConstants.DataQueueLength];
        private XYPoint[] dataFFTSingleSided;
        private int dataFFTSingleSidedLength = 0;

        // Constructors
        private PediatricSensor() { }
        public PediatricSensor(string serial)
        {
            PediatricSensorConstructorMethod(serial);

            InitializeDataFFTSingleSided();
        }

        // Properties

        private PediatricSensorConfig pediatricSensorConfig;
        public PediatricSensorConfig PediatricSensorConfig
        {
            get { return pediatricSensorConfig; }
            set { pediatricSensorConfig = value; }
        }

        private PediatricSoftConstants.SensorState state = PediatricSoftConstants.SensorState.Init;
        public PediatricSoftConstants.SensorState State
        {
            get { return state; }
            private set
            {
                state = value;
                RaisePropertyChanged();
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Entering state {value}");
            }
        }

        private readonly ConcurrentQueue<string> commandHistory = new ConcurrentQueue<string>();
        public ConcurrentQueue<string> CommandHistory
        {
            get { return commandHistory; }
        }

        public string Transmission
        {
            get
            {
                double result = Math.Round(1000 * lastDataPoint.ADC / coldSensorADCValue) / 10;
                return string.Concat(result.ToString(), "%");
            }
        }

        public int LastValueRAW
        {
            get
            {
                switch (PediatricSensorData.DataSelect)
                {
                    case PediatricSoftConstants.DataSelect.ADC:
                        return lastDataPoint.ADCRAW;

                    case PediatricSoftConstants.DataSelect.OpenLoop:
                        return lastDataPoint.BzDemodRAW;

                    case PediatricSoftConstants.DataSelect.ClosedLoop:
                        return lastDataPoint.BzErrorRAW;

                    default:
                        return 0;
                }
            }
        }

        public string LastValueDisplay
        {
            get
            {
                switch (PediatricSensorData.DataSelect)
                {
                    case PediatricSoftConstants.DataSelect.ADC:
                        return Math.Round(lastDataPoint.ADC, 3, MidpointRounding.AwayFromZero).ToString();

                    case PediatricSoftConstants.DataSelect.OpenLoop:
                        return lastDataPoint.BzDemod.ToString("+0.000E+00;-0.000E+00");

                    case PediatricSoftConstants.DataSelect.ClosedLoop:
                        return lastDataPoint.BzError.ToString("+0.000E+00;-0.000E+00");

                    default:
                        return "";
                }
            }
        }

        public double LastValue
        {
            get
            {
                switch (PediatricSensorData.DataSelect)
                {
                    case PediatricSoftConstants.DataSelect.ADC:
                        return lastDataPoint.ADC;

                    case PediatricSoftConstants.DataSelect.OpenLoop:
                        return lastDataPoint.BzDemod;

                    case PediatricSoftConstants.DataSelect.ClosedLoop:
                        return lastDataPoint.BzError;

                    default:
                        return 0;
                }
            }
        }

        public ushort Chassis { get { return PediatricSensorConfig.Chassis; } }
        public ushort Port { get { return PediatricSensorConfig.Port; } }
        public ushort Head { get { return PediatricSensorConfig.Head; } }

        public string SN { get; private set; } = String.Empty;
        public string PortSN { get { return String.Concat(SN); } }
        public bool CanSendCommand { get; private set; } = true;
        public bool IsDisposed { get; private set; } = false;
        public bool IsLocked { get; private set; } = false;
        public bool IsRunning { get; private set; } = false;
        public bool IsValid { get; private set; } = false;

        private bool isPlotted = false;
        public bool IsPlotted
        {
            get { return isPlotted; }
            set
            {
                ChartValues.Clear();
                isPlotted = value;
                InitializeDataFFTSingleSided();
                RaisePropertyChanged();
                PediatricSensorData.UpdateSeriesCollection();
            }
        }

        public ChartValues<double> ChartValues { get; private set; } = new ChartValues<double>();
        public ChartValues<XYPoint> ChartValuesFFT { get; private set; } = new ChartValues<XYPoint>();

        // Methods
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
                while (currentState != PediatricSoftConstants.SensorState.LaserLockDone &&
                       currentState != PediatricSoftConstants.SensorState.Failed)
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
                else
                {
                    if (PediatricSensorData.DebugMode)
                    {
                        State = PediatricSoftConstants.SensorState.Start;
                        currentState = State;
                        canRun = true;
                    }
                }
            }

            if (canRun)
                while (currentState != PediatricSoftConstants.SensorState.Run &&
                       currentState != PediatricSoftConstants.SensorState.Failed)
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
                while (currentState != PediatricSoftConstants.SensorState.Idle &&
                       currentState != PediatricSoftConstants.SensorState.Failed)
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
                while (currentState != PediatricSoftConstants.SensorState.Idle &&
                       currentState != PediatricSoftConstants.SensorState.Failed)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }
        }

        public void SendCommand(string command)
        {
            command = Regex.Replace(command, @"[^\w@#?+-]", "");

            if (!String.IsNullOrEmpty(command))
            {
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Sending command \"{command}\"");
                commandQueue.Enqueue(command + "\n");
                CommandHistory.Enqueue(command);
                RaisePropertyChanged("CommandHistory");
            }
        }

        public static string UInt16ToStringBE(ushort value)
        {
            byte[] t = BitConverter.GetBytes(value);
            Array.Reverse(t);
            string s = BitConverter.ToString(t);
            s = Regex.Replace(s, @"[^\w]", "");
            return s;
        }

        private static ushort UShortSafeInc(ushort value, ushort step, ushort max)
        {
            int newValue = value + step;
            if (newValue < max)
                return (ushort)newValue;
            else return max;
        }

        private static ushort UShortSafeDec(ushort value, ushort step, ushort min)
        {
            int newValue = value - step;
            if (newValue > min)
                return (ushort)newValue;
            else return min;
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
                int dataCounter = 0;

                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Streaming starting");

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
                            PediatricSensorData.DebugLogQueue.Enqueue($"Failed to write to FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
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
                        PediatricSensorData.DebugLogQueue.Enqueue($"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
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
                        PediatricSensorData.DebugLogQueue.Enqueue($"Failed to read in the Rx buffer on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                        break;
                    }

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

                    // If we read more than 0 bytes, we run the processing logic on the buffer and reset the bytesRead at the end.
                    if (bytesRead > 0)
                    {
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
                                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Info message: {infoMessage}");
                                CommandHistory.Enqueue(infoMessage);
                                RaisePropertyChanged("CommandHistory");
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

                                    lastDataPoint = new DataPoint
                                    (
                                        BitConverter.ToInt32(data, 12), // TimeRAW
                                        BitConverter.ToInt32(data, 8), // ADCRAW
                                        BitConverter.ToInt32(data, 4), // BzDemod
                                        BitConverter.ToInt32(data, 0), // BzError
                                        PediatricSoftConstants.ConversionTime * BitConverter.ToInt32(data, 12),
                                        PediatricSoftConstants.ConversionADC * BitConverter.ToInt32(data, 8),
                                        CalibrationBzDemod * BitConverter.ToInt32(data, 4),
                                        PediatricSoftConstants.SensorCoilsCalibrationTeslaPerHex * (0x8000 - BitConverter.ToInt32(data, 0))
                                    );

                                    Array.Copy(dataPoints, 1, dataPoints, 0, PediatricSoftConstants.DataQueueLength - 1);
                                    dataPoints[PediatricSoftConstants.DataQueueLength - 1] = lastDataPoint;

                                    dataCounter++;
                                    if (dataCounter == PediatricSoftConstants.FFTLength)
                                    {
                                        if (isPlotted)
                                        {
                                            Complex[] dataFFTComplex = new Complex[PediatricSoftConstants.FFTLength];
                                            for (int i_dataFFTComplex = 0; i_dataFFTComplex < PediatricSoftConstants.FFTLength; i_dataFFTComplex++)
                                            {
                                                switch (PediatricSensorData.DataSelect)
                                                {
                                                    case PediatricSoftConstants.DataSelect.ADC:
                                                        dataFFTComplex[i_dataFFTComplex] = new Complex(dataPoints[PediatricSoftConstants.DataQueueLength - PediatricSoftConstants.FFTLength + i_dataFFTComplex].ADC, 0);
                                                        break;

                                                    case PediatricSoftConstants.DataSelect.OpenLoop:
                                                        dataFFTComplex[i_dataFFTComplex] = new Complex(dataPoints[PediatricSoftConstants.DataQueueLength - PediatricSoftConstants.FFTLength + i_dataFFTComplex].BzDemod, 0);
                                                        break;

                                                    case PediatricSoftConstants.DataSelect.ClosedLoop:
                                                        dataFFTComplex[i_dataFFTComplex] = new Complex(dataPoints[PediatricSoftConstants.DataQueueLength - PediatricSoftConstants.FFTLength + i_dataFFTComplex].BzError, 0);
                                                        break;
                                                }

                                            }

                                            Fourier.Forward(dataFFTComplex, FourierOptions.Matlab);
                                            
                                            dataFFTSingleSided[0].Y = dataFFTComplex[0].Magnitude / PediatricSoftConstants.FFTLength;
                                            for (int i_dataFFTSingleSided = 1; i_dataFFTSingleSided < dataFFTSingleSidedLength; i_dataFFTSingleSided++)
                                            {
                                                dataFFTSingleSided[i_dataFFTSingleSided].Y = 2 * dataFFTComplex[i_dataFFTSingleSided].Magnitude / PediatricSoftConstants.FFTLength;
                                            }
                                            
                                            ChartValuesFFT.Clear();
                                            ChartValuesFFT.AddRange(dataFFTSingleSided);
                                        }

                                        dataCounter = 0;
                                    }

                                    if (isPlotted)
                                    {
                                        plotCounter++;
                                        if (plotCounter == PediatricSoftConstants.DataQueueLength / PediatricSoftConstants.PlotQueueLength)
                                        {
                                            ChartValues.Add(LastValue);
                                            if (ChartValues.Count > PediatricSoftConstants.PlotQueueLength) ChartValues.RemoveAt(0);
                                            plotCounter = 0;
                                        }
                                    }

                                    if (dataSaveEnable)
                                        if (dataSaveRAW)
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(lastDataPoint.TimeRAW), "\t", Convert.ToString(LastValueRAW)));
                                        else
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(lastDataPoint.Time), "\t", Convert.ToString(LastValue)));

                                    if (!dataUpdated) dataUpdated = true;
                                }

                                dataIndex = 0;
                            }

                        }

                        // Reset bytesRead to 0
                        bytesRead = 0;

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

                    // Sleep a bit
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);

                    // Update the current State
                    lock (stateLock)
                    {
                        currentState = State;
                    }

                }

                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Streaming stoping");

            });
        }

        private Task StateHandlerAsync()
        {
            return Task.Run(() =>
            {
                PediatricSoftConstants.SensorState currentState;

                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: State handler: started");

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
                                State++;
                            }
                            break;

                        case PediatricSoftConstants.SensorState.CalibrateMagnetometer:
                            SendCommandsCalibrateMagnetometer();
                            break;

                        case PediatricSoftConstants.SensorState.CellHeatLock:
                            SendCommandsCellHeatLock();
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

                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: State handler: exiting");

            });
        }

        private void PediatricSensorConstructorMethod(string serial)
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            _FTDI = new FTDI();

            SN = serial;

            configPath = System.IO.Path.Combine(PediatricSensorData.Instance.SensorConfigFolderAbsolute, SN) + ".xml";
            LoadConfig();

            for (int i = 0; i < PediatricSoftConstants.SerialPortMaxRetries; i++)
            {

                // Open the device by serial number
                ftStatus = _FTDI.OpenBySerialNumber(serial);
                if (ftStatus != FTDI.FT_STATUS.FT_OK)
                {
                    lock (stateLock)
                    {
                        State = PediatricSoftConstants.SensorState.Failed;
                    }
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to open FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                    Dispose();
                    return;
                }

                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Opened FTDI device with S/N {SN}");

                // Set Baud rate
                ftStatus = _FTDI.SetBaudRate(PediatricSoftConstants.SerialPortBaudRate);
                if (ftStatus != FTDI.FT_STATUS.FT_OK)
                {
                    lock (stateLock)
                    {
                        State = PediatricSoftConstants.SensorState.Failed;
                    }
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to set Baud rate on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to set data characteristics on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to set flow control on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to set timeouts on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to set latency on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to purge buffers on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to write to FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
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
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to purge buffers on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                    Dispose();
                    return;
                }

                if (numBytes > 0)
                {
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Valid");
                    lock (stateLock)
                    {
                        State = PediatricSoftConstants.SensorState.Valid;
                    }
                    IsValid = true;
                    return;
                }

                // Reset the device if we got no response
                ftStatus = _FTDI.ResetDevice();
                if (ftStatus != FTDI.FT_STATUS.FT_OK)
                {
                    lock (stateLock)
                    {
                        State = PediatricSoftConstants.SensorState.Failed;
                    }
                    PediatricSensorData.DebugLogQueue.Enqueue($"Failed to reset the FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                    Dispose();
                    return;
                }

                // Sleep a bit
                Thread.Sleep(PediatricSoftConstants.SerialPortSleepAfterFail);

            }

            PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Didn't get a response from the board: not valid");
            Dispose();

        }

        private void SaveConfig()
        {
            if (pediatricSensorConfig.Equals(pediatricSensorConfigOnLoad))
            {
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Configuration not changed - not saving");
            }
            else
            {
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Saving configuration");

                TextWriter writer = null;
                try
                {
                    XmlSerializer serializer = new XmlSerializer(typeof(PediatricSensorConfig));
                    writer = new StreamWriter(configPath);
                    serializer.Serialize(writer, pediatricSensorConfig);
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
            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Loading configuration");

            TextReader reader = null;
            try
            {
                XmlSerializer serializer = new XmlSerializer(typeof(PediatricSensorConfig));
                reader = new StreamReader(configPath);
                pediatricSensorConfig = (PediatricSensorConfig)serializer.Deserialize(reader);
            }
            catch (Exception)
            {
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Failed to load configuration - using defaults");
                pediatricSensorConfig = new PediatricSensorConfig();
            }
            finally
            {
                if (reader != null)
                    reader.Close();
            }

            pediatricSensorConfigOnLoad = pediatricSensorConfig.GetValueCopy();
        }

        public void SendCommandsSetup()
        {
            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLockDisable)));

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

            SendCommand(PediatricSoftConstants.SensorCommandPIDLaserCurrentP);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDLaserCurrentP)));

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
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultDigitalDataSelector)));

            SendCommand(PediatricSoftConstants.SensorCommandADCDisplayGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultADCDisplayGain)));

            SendCommand(PediatricSoftConstants.SensorCommandStepSize);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultStepSize)));

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

            double transmission = double.MaxValue;

            // Turn on the laser
            lock (dataLock)
            {
                SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
                SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.LaserCurrent)));
                dataUpdated = false;
            }

            // Wait a bit
            Thread.Sleep(PediatricSoftConstants.StateHandlerLaserHeatSweepTime);

            // Record the ADC value
            coldSensorADCValue = lastDataPoint.ADC;

            // Here we check if the laser is working as expected
            if (coldSensorADCValue < PediatricSoftConstants.SensorADCColdValueLowGainMinVolts ||
                coldSensorADCValue > PediatricSoftConstants.SensorADCColdValueLowGainMaxVolts)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN} failed: ADC value out of range ({coldSensorADCValue} V)");
                return;
            }

            // Set the cell heater to the max value
            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.MaxCellHeat)));

            // Wait for the cell to warm up
            Thread.Sleep(PediatricSoftConstants.StateHandlerCellHeatInitialTime);

            // Turn on the Z-coil to increase absorption
            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockFieldZOffset)));

            // Laser heat sweep cycle. Here we look for the Rb resonance.

            for (int i = 0; i < PediatricSoftConstants.MaxNumberOfLaserLockSweepCycles; i++)
            {
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Laser lock sweep cycle: {i}");

                // Set the laser heater to the max value
                SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMaxLaserHeat)));

                // Wait a bit
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Waiting for {PediatricSoftConstants.StateHandlerLaserHeatSweepTime} ms");
                Thread.Sleep(PediatricSoftConstants.StateHandlerLaserHeatSweepTime);

                // Set the laser heater to the min value
                SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMinLaserHeat)));

                // Wait a bit
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Waiting for {PediatricSoftConstants.StateHandlerLaserHeatSweepTime} ms");
                Thread.Sleep(PediatricSoftConstants.StateHandlerLaserHeatSweepTime);

                // See if the threshold was reached
                lock (dataLock)
                {
                    transmission = dataPoints.Select(x => x.ADC).Min() / coldSensorADCValue;
                }

                if (transmission < PediatricSoftConstants.SensorTargetLaserTransmissionSweep)
                {
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Sweep cycle - found Rb resonance");
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Transmission on resonance: {transmission}");

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
                            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Lock was aborted or something failed. Returning.");
                            return;
                        }
                    }
                }

                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Didn't find Rb resonance, going to wait more");
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Minimal transmission {transmission} is higher than the target {PediatricSoftConstants.SensorTargetLaserTransmissionSweep}");

            }

            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Didn't find Rb resonance, giving up");

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
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Laser lock step cycle: {i}");

                while (laserHeat < PediatricSoftConstants.SensorMaxLaserHeat)
                {
                    while (!dataUpdated) Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                    transmission = lastDataPoint.ADC / coldSensorADCValue;

                    if (transmission < PediatricSoftConstants.SensorTargetLaserTransmissionStep)
                    {
                        if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Step cycle - found Rb resonance");
                        if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Transmission on resonance: {transmission}");

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
                                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Lock was aborted or something failed. Returning.");
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
                    State = PediatricSoftConstants.SensorState.Failed;
                }
            }
            PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: We saw Rb resonance, but the step cycle failed. Giving up");

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

            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndADCGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataStreamingOnGainHigh)));

            coldSensorADCValue = coldSensorADCValue * 50 / 15.5;

            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockEnable)));

            //SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
            //SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserHeat)));

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State++;
                }
                else
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Lock was aborted or something failed. Returning.");
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
                    transmission = lastDataPoint.ADC / coldSensorADCValue;
                }
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorConfig.DefaultCellHeat)));

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State++;
                }
                else
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Lock was aborted or something failed. Returning.");
            }
        }

        private void SendCommandsZeroFields()
        {
            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Begin field zeroing");

            PediatricSoftConstants.SensorState currentState;
            PediatricSoftConstants.SensorState correctState;

            lock (stateLock)
            {
                currentState = State;
                correctState = State;
            }

            ushort fieldStep = ushort.MaxValue / PediatricSoftConstants.NumberOfFieldZeroingIntervalsOneAxis - 1;
            int maxADCRAWValue = int.MinValue;
            int currentADCRAWValue = int.MinValue;

            // Reset fields

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

            for (ushort yField = ushort.MinValue; yField < ushort.MaxValue; yField = UShortSafeInc(yField, fieldStep, ushort.MaxValue))
            {
                for (ushort zField = ushort.MinValue; zField < ushort.MaxValue; zField = UShortSafeInc(zField, fieldStep, ushort.MaxValue))
                {
                    SendCommand(PediatricSoftConstants.SensorCommandFieldYOffset);
                    SendCommand(String.Concat("#", UInt16ToStringBE(yField)));

                    SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
                    SendCommand(String.Concat("#", UInt16ToStringBE(zField)));

                    while (commandQueue.TryPeek(out string dummy))
                    {
                        Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        if (State != correctState)
                        {
                            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                            return;
                        }
                    }

                    lock (dataLock)
                    {
                        dataUpdated = false;
                    }

                    while (!dataUpdated)
                    {
                        Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        if (State != correctState)
                        {
                            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                            return;
                        }
                    }

                    currentADCRAWValue = lastDataPoint.ADCRAW;

                    if (currentADCRAWValue > maxADCRAWValue)
                    {
                        maxADCRAWValue = currentADCRAWValue;

                        zeroYField = yField;
                        zeroZField = zField;

                    }

                }

            }

            SendCommand(PediatricSoftConstants.SensorCommandFieldYOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(zeroYField)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(zeroZField)));

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State++;
                }
                else
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
            }

            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Field zeroing done");
        }

        private void SendCommandsCalibrateMagnetometer()
        {
            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Begin calibration");

            PediatricSoftConstants.SensorState currentState;
            PediatricSoftConstants.SensorState correctState;

            lock (stateLock)
            {
                currentState = State;
                correctState = State;
            }

            ushort plusField = UShortSafeInc(zeroZField, PediatricSoftConstants.SensorFieldStep, ushort.MaxValue);
            ushort minusField = UShortSafeDec(zeroZField, PediatricSoftConstants.SensorFieldStep, ushort.MinValue);

            double plusValue;
            double minusValue;

            SendCommand(PediatricSoftConstants.SensorCommandFieldZModulationAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorConfig.FieldZModulationAmplitude)));

            SendCommand(PediatricSoftConstants.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(plusField)));

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            Thread.Sleep(PediatricSoftConstants.StateHandlerCellHeatInitialTime);

            lock (dataLock)
            {
                plusValue = dataPoints.Select(x => x.BzDemodRAW).Average();
            }

            SendCommand(String.Concat("#", UInt16ToStringBE(minusField)));

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            Thread.Sleep(PediatricSoftConstants.StateHandlerCellHeatInitialTime);

            lock (dataLock)
            {
                minusValue = dataPoints.Select(x => x.BzDemodRAW).Average();
            }

            SendCommand(String.Concat("#", UInt16ToStringBE(zeroZField)));

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            CalibrationBzDemod = PediatricSoftConstants.SensorCoilsCalibrationTeslaPerHex / ((plusValue - minusValue) / (plusField - minusField));

            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: ZDemod calibration {CalibrationBzDemod}");

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State++;
                }
                else
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
            }
        }

        private void SendCommandsCellHeatLock()
        {
            PediatricSoftConstants.SensorState currentState;
            PediatricSoftConstants.SensorState correctState;

            lock (stateLock)
            {
                currentState = State;
                correctState = State;
            }

            double targetADCValue = coldSensorADCValue * PediatricSoftConstants.SensorTargetLaserTransmissionRun;
            if (targetADCValue > PediatricSoftConstants.SensorADCColdValueLowGainMaxVolts)
            {
                targetADCValue = PediatricSoftConstants.SensorADCColdValueLowGainMaxVolts;
            }
            ushort cellHeatLockPoint = (ushort)(ushort.MaxValue * (targetADCValue / 5));

            SendCommand(PediatricSoftConstants.SensorCommandCellHeatLockPoint);
            SendCommand(String.Concat("#", UInt16ToStringBE(cellHeatLockPoint)));

            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLaserLockEnable | PediatricSoftConstants.SensorCellLockEnable)));

            Thread.Sleep(PediatricSoftConstants.StateHandlerCellHeatLockSleepTime);

            lock (stateLock)
            {
                currentState = State;
                if (currentState == correctState)
                {
                    State++;
                }
                else
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
            }

        }

        public void Dispose()
        {
            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Dispose() was called");

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
                if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Sensor {SN}: Disposing the UI timer");
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

        public void InitializeDataFFTSingleSided()
        {
            double[] frequencyScale = Fourier.FrequencyScale(PediatricSoftConstants.FFTLength, PediatricSoftConstants.DataSampleRate);

            for (int i = 0; i < PediatricSoftConstants.FFTLength; i++)
            {
                if (frequencyScale[i] >= PediatricSoftConstants.FFTMaxFrequency)
                {
                    dataFFTSingleSidedLength = i + 1;
                    break;
                }
            }

            if (dataFFTSingleSidedLength == 0)
            {
                dataFFTSingleSidedLength = PediatricSoftConstants.FFTLength / 2;
            }

            lock (dataLock)
            {
                dataFFTSingleSided = new XYPoint[dataFFTSingleSidedLength];

                for (int i = 0; i < dataFFTSingleSidedLength; i++)
                {
                    dataFFTSingleSided[i] = new XYPoint(frequencyScale[i], 0);
                }
            }
        }

        ~PediatricSensor()
        {
            if (!IsDisposed) Dispose();
        }

        // Event Handlers
        private void OnUIUpdateTimerEvent(Object source, ElapsedEventArgs e)
        {
            RaisePropertyChanged("LastValueRAW");
            RaisePropertyChanged("LastValueDisplay");
            RaisePropertyChanged("Transmission");
        }

    }
}