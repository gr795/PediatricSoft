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
        private DebugLog DebugLog = DebugLog.Instance;
        private PediatricSensorConfig pediatricSensorConfigOnLoad;
        private readonly string configPath;

        private FTDI _FTDI;
        private Task streamingTask;
        private Task stateHandlerTask;
        private System.Timers.Timer uiUpdateTimer;

        private readonly Object stateLock = new Object();
        private readonly Object dataLock = new Object();

        private double coldSensorADCValue = 0;
        private ushort currentCellHeat = 0;
        private ushort zeroBx = PediatricSoftConstants.SensorDefaultBxOffset;
        private ushort zeroBy = PediatricSoftConstants.SensorDefaultByOffset;
        private ushort zeroBz = PediatricSoftConstants.SensorDefaultBzOffset;

        private bool infoRequested = false;
        private string requestedInfoString = string.Empty;

        private ConcurrentQueue<string> commandQueue = new ConcurrentQueue<string>();

        private DataPoint lastDataPoint;

        private bool dataUpdated = false;

        private bool dataSaveEnable = false;
        private bool dataSaveRAW = true;

        private List<string> dataSaveBuffer = new List<string>();
        private string filePath = string.Empty;

        private DataPoint[] dataPoints = new DataPoint[PediatricSoftConstants.DataQueueLength];
        private XYPoint[] dataFFTSingleSided;
        private int dataFFTSingleSidedLength = 0;
        private double dataFFTFrequencyStep;
        private double[] dataFFTWindow;
        private double dataFFTWindowAmplitudeFactor;
        private double dataFFTWindowNoisePowerBandwidth;

        // Constructors
        private PediatricSensor() { }
        public PediatricSensor(string serial)
        {
            PediatricSensorConstructorMethod(serial);
            if (IsValid)
            {
                InitializeDataFFTSingleSided();
                SN = serial;
                configPath = System.IO.Path.Combine(PediatricSensorData.Instance.SensorConfigFolderAbsolute, SN) + ".xml";
                LoadConfig();
            }
            else
            {
                _FTDI.GetCOMPort(out string comPort);
                DebugLog.Enqueue($"Sensor {serial}: Didn't get a response from the board. Windows port name {comPort}.");
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                Dispose();
            }
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
                if (PediatricSensorData.DebugMode)
                {
                    DebugLog.Enqueue($"Sensor {SN}: Entering state {value}");
                }
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
                double transmission = lastDataPoint.ADC / coldSensorADCValue;
                if (transmission >= 0 && transmission < double.MaxValue)
                {
                    double result = Math.Round(1000 * transmission) / 10;
                    return string.Concat(result.ToString(), "%");
                }
                else
                {
                    return string.Empty;
                }
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
                        return lastDataPoint.BzFeedbackRAW;

                    case PediatricSoftConstants.DataSelect.DoubleF:
                        return lastDataPoint.Bz2fRAW;

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
                        return lastDataPoint.BzFeedback.ToString("+0.000E+00;-0.000E+00");

                    case PediatricSoftConstants.DataSelect.DoubleF:
                        return lastDataPoint.Bz2f.ToString("+0.000E+00;-0.000E+00");

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
                        return lastDataPoint.BzFeedback;

                    case PediatricSoftConstants.DataSelect.DoubleF:
                        return lastDataPoint.Bz2f;

                    default:
                        return 0;
                }
            }
        }

        private double calibrationBzDemod = 0;
        public double CalibrationBzDemod
        {
            get
            {
                return calibrationBzDemod;
            }
            private set
            {
                calibrationBzDemod = value;
                RaisePropertyChanged();
                RaisePropertyChanged("CalibrationBzDemodString");
            }
        }

        public string CalibrationBzDemodString
        {
            get
            {
                return CalibrationBzDemod.ToString("+0.000E+00;-0.000E+00");
            }
        }

        public string Name { get { return PediatricSensorConfig.Name; } }

        public ushort Chassis { get { return PediatricSensorConfig.Chassis; } }
        public ushort Port { get { return PediatricSensorConfig.Port; } }
        public ushort Head { get { return PediatricSensorConfig.Head; } }

        public string SN { get; private set; } = String.Empty;
        public string UserFriendlyName
        {
            get
            {
                return $"C{Chassis}-P{Port}-{SN}";
            }
        }

        public bool CanSendCommand { get; private set; } = true;
        public bool IsDisposed { get; private set; } = false;
        public bool IsLocked { get; private set; } = false;
        public bool IsRunning { get; private set; } = false;
        public bool IsValid { get; private set; } = false;
        public bool IsMasterCard
        {
            get
            {
                return (pediatricSensorConfig.Chassis == PediatricSoftConstants.MasterCardChassis &&
                        pediatricSensorConfig.Port == PediatricSoftConstants.MasterCardPort);
            }
        }

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

        public void Standby()
        {
            PediatricSoftConstants.SensorState currentState;
            bool canRun = false;

            lock (stateLock)
            {
                currentState = State;

                if (currentState < PediatricSoftConstants.SensorState.Start)
                {
                    State = PediatricSoftConstants.SensorState.Setup;
                    currentState = State;
                    canRun = true;
                }
                else
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                    DebugLog.Enqueue($"Sensor {SN}: Standby procedure was called, but we shouldn't be here. Failing..");
                }
            }

            if (canRun)
            {
                while (currentState != PediatricSoftConstants.SensorState.Standby &&
                       currentState != PediatricSoftConstants.SensorState.Failed)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }
            }
        }

        public void Lock()
        {
            PediatricSoftConstants.SensorState currentState;
            bool canRun = false;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.Standby ||
                    currentState == PediatricSoftConstants.SensorState.Idle)
                {
                    State = PediatricSoftConstants.SensorState.StartLock;
                    currentState = State;
                    canRun = true;
                }
                else
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                    DebugLog.Enqueue($"Sensor {SN}: Lock procedure was called, but we shouldn't be here. Failing..");
                }
            }

            if (canRun)
            {
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
            {
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
        }

        public void Stop()
        {
            PediatricSoftConstants.SensorState currentState;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.Run)
                {
                    State++;
                }
            }

            if (currentState == PediatricSoftConstants.SensorState.Run)
            {
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
        }

        public void ZeroFields()
        {
            PediatricSoftConstants.SensorState currentState;
            bool canRun = false;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.Idle)
                {
                    State = PediatricSoftConstants.SensorState.StartFieldZeroing;
                    currentState = State;
                    canRun = true;
                }
            }

            if (canRun)
            {
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
        }

        public void SwitchMagnetometerMode()
        {
            PediatricSoftConstants.SensorState currentState;
            bool canRun = false;

            lock (stateLock)
            {
                currentState = State;

                if (currentState == PediatricSoftConstants.SensorState.Idle)
                {
                    State = PediatricSoftConstants.SensorState.SwitchMagnetometerMode;
                    currentState = State;
                    canRun = true;
                }
            }

            if (canRun)
            {
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
        }

        public void SendCommand(string command)
        {
            command = Regex.Replace(command, @"[^\w@#?+-]", "");

            if (!String.IsNullOrEmpty(command))
            {
                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Sending command \"{command}\"");
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

                int plotCounter = 0;
                int dataCounter = 0;

                int _TimeRAW = 0;
                int _ADCRAW = 0;
                int _BzDemodRAW = 0;
                int _BzFeedbackRAW = 0;
                int _Bz2fRAW = 0;

                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Streaming starting");

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
                            DebugLog.Enqueue($"Failed to write to FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
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
                        DebugLog.Enqueue($"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
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
                        DebugLog.Enqueue($"Failed to read in the Rx buffer on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                        break;
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
                                if (PediatricSensorData.DebugMode)
                                {
                                    DebugLog.Enqueue($"Sensor {SN}: Info message: {infoMessage}");
                                }
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

                                    _TimeRAW = BitConverter.ToInt32(data, 16);
                                    _ADCRAW = BitConverter.ToInt32(data, 0);
                                    _BzDemodRAW = BitConverter.ToInt32(data, 4);
                                    _BzFeedbackRAW = BitConverter.ToInt32(data, 12);
                                    _Bz2fRAW = BitConverter.ToInt32(data, 8);


                                    lastDataPoint = new DataPoint
                                    (
                                        _TimeRAW,
                                        _ADCRAW,
                                        _BzDemodRAW,
                                        _BzFeedbackRAW,
                                        _Bz2fRAW,
                                        PediatricSoftConstants.ConversionTime * _TimeRAW,
                                        PediatricSoftConstants.ConversionADC * _ADCRAW,
                                        CalibrationBzDemod * _BzDemodRAW,
                                        PediatricSoftConstants.SensorZCoilCalibrationTeslaPerHex * (_BzFeedbackRAW / PediatricSoftConstants.FeedbackStreamingScalingFactor),
                                        _Bz2fRAW
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
                                                double newRealPoint = 0;
                                                switch (PediatricSensorData.DataSelect)
                                                {
                                                    case PediatricSoftConstants.DataSelect.ADC:
                                                        newRealPoint = dataFFTWindow[i_dataFFTComplex] * dataFFTWindowAmplitudeFactor * dataPoints[PediatricSoftConstants.DataQueueLength - PediatricSoftConstants.FFTLength + i_dataFFTComplex].ADC;
                                                        break;

                                                    case PediatricSoftConstants.DataSelect.OpenLoop:
                                                        newRealPoint = dataFFTWindow[i_dataFFTComplex] * dataFFTWindowAmplitudeFactor * dataPoints[PediatricSoftConstants.DataQueueLength - PediatricSoftConstants.FFTLength + i_dataFFTComplex].BzDemod;
                                                        break;

                                                    case PediatricSoftConstants.DataSelect.ClosedLoop:
                                                        newRealPoint = dataFFTWindow[i_dataFFTComplex] * dataFFTWindowAmplitudeFactor * dataPoints[PediatricSoftConstants.DataQueueLength - PediatricSoftConstants.FFTLength + i_dataFFTComplex].BzFeedback;
                                                        break;
                                                }
                                                dataFFTComplex[i_dataFFTComplex] = new Complex(newRealPoint, 0);
                                            }

                                            Fourier.Forward(dataFFTComplex, FourierOptions.Matlab);

                                            dataFFTSingleSided[0].Y = dataFFTComplex[0].Magnitude / PediatricSoftConstants.FFTLength;
                                            for (int i_dataFFTSingleSided = 1; i_dataFFTSingleSided < dataFFTSingleSidedLength; i_dataFFTSingleSided++)
                                            {
                                                dataFFTSingleSided[i_dataFFTSingleSided].Y = 2 * dataFFTComplex[i_dataFFTSingleSided].Magnitude / PediatricSoftConstants.FFTLength / dataFFTWindowNoisePowerBandwidth / Math.Sqrt(dataFFTFrequencyStep);
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
                                    {
                                        if (dataSaveRAW)
                                        {
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(lastDataPoint.TimeRAW), "\t", Convert.ToString(LastValueRAW)));
                                        }
                                        else
                                        {
                                            dataSaveBuffer.Add(String.Concat(Convert.ToString(lastDataPoint.Time), "\t", Convert.ToString(LastValue)));
                                        }
                                    }

                                    if (!dataUpdated) dataUpdated = true;
                                }

                                dataIndex = 0;
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

                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Streaming stoping");

            });
        }

        private Task StateHandlerAsync()
        {
            return Task.Run(() =>
            {
                PediatricSoftConstants.SensorState currentState;

                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: State handler: started");

                lock (stateLock)
                {
                    currentState = State;
                }

                while (currentState < PediatricSoftConstants.SensorState.ShutDownComplete)
                {
                    switch (currentState)
                    {
                        case PediatricSoftConstants.SensorState.Valid:
                            if (PediatricSensorData.DebugMode)
                            {
                                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                            }
                            else
                            {
                                lock (stateLock)
                                {
                                    State++;
                                }
                            }
                            break;

                        case PediatricSoftConstants.SensorState.Setup:
                            SendCommandsSetup(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.StartLock:
                            SendCommandsStartLock(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.LaserLockStep:
                            SendCommandsLaserLockStep(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.LaserLockPID:
                            SendCommandsLaserLockPID(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.CellHeatLock:
                            SendCommandsCellHeatLock(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.StabilizeCellHeat:
                            SendCommandsStabilizeCellHeat(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.StartFieldZeroing:
                            lock (stateLock)
                            {
                                State++;
                            }
                            break;

                        case PediatricSoftConstants.SensorState.HoldCurrentCellHeat:
                            SendCommandsHoldCurrentCellHeat(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.ZeroFieldsYZ:
                            SendCommandsZeroFieldsYZ(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.HoldCurrentBy:
                            SendCommandsHoldCurrentBy(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.HoldCurrentBz:
                            SendCommandsHoldCurrentBz(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.ZeroFieldX:
                            SendCommandsZeroFieldX(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.HoldCurrentTransmission:
                            SendCommandsHoldCurrentTransmission(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.SwitchMagnetometerMode:
                            SendCommandsSwitchMagnetometerMode(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.Start:
                            lock (stateLock)
                            {
                                State++;
                            }
                            break;

                        case PediatricSoftConstants.SensorState.StartDataSave:
                            SendCommandsStartDataSave(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.SyncTimers:
                            SendCommandsSyncTimers(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.Stop:
                            lock (stateLock)
                            {
                                State++;
                            }
                            break;

                        case PediatricSoftConstants.SensorState.StopDataSave:
                            SendCommandsStopDataSave(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.GoIdle:
                            lock (stateLock)
                            {
                                State = PediatricSoftConstants.SensorState.Idle;
                            }
                            break;

                        case PediatricSoftConstants.SensorState.SoftFail:
                            SendCommandsShutDown(currentState);
                            break;

                        case PediatricSoftConstants.SensorState.ShutDownRequested:
                            SendCommandsShutDown(currentState);
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

                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: State handler: exiting");

            });
        }

        private void PediatricSensorConstructorMethod(string serial)
        {
            FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

            _FTDI = new FTDI();

            const string checkString = "?\n";
            UInt32 numBytes = 0;

            // Open the device by serial number
            ftStatus = _FTDI.OpenBySerialNumber(serial);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to open FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                return;
            }

            // Set Baud rate
            ftStatus = _FTDI.SetBaudRate(PediatricSoftConstants.SerialPortBaudRate);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to set Baud rate on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                return;
            }

            // Set data characteristics - Data bits, Stop bits, Parity
            ftStatus = _FTDI.SetDataCharacteristics(FTDI.FT_DATA_BITS.FT_BITS_8, FTDI.FT_STOP_BITS.FT_STOP_BITS_1, FTDI.FT_PARITY.FT_PARITY_NONE);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to set data characteristics on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                return;
            }

            // Set flow control - set RTS/CTS flow control - we don't have flow control
            ftStatus = _FTDI.SetFlowControl(FTDI.FT_FLOW_CONTROL.FT_FLOW_NONE, 0x11, 0x13);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to set flow control on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                return;
            }

            // Set timeouts
            ftStatus = _FTDI.SetTimeouts(PediatricSoftConstants.SerialPortReadTimeout, PediatricSoftConstants.SerialPortWriteTimeout);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to set timeouts on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                return;
            }

            // Set latency
            ftStatus = _FTDI.SetLatency(PediatricSoftConstants.SerialPortLatency);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to set latency on FTDI device with S/N {serial}. Error: {ftStatus.ToString()}");
                return;
            }

            // Purge port buffers
            ftStatus = _FTDI.Purge(FTDI.FT_PURGE.FT_PURGE_RX + FTDI.FT_PURGE.FT_PURGE_TX);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to purge buffers on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                return;
            }

            // Write ? into the port to see if we have a working board
            ftStatus = _FTDI.Write(checkString, checkString.Length, ref numBytes);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to write to FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                return;
            }

            // Wait a bit
            Thread.Sleep((int)PediatricSoftConstants.SerialPortWriteTimeout);

            // Check if the board responded
            ftStatus = _FTDI.GetRxBytesAvailable(ref numBytes);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to get number of available bytes in Rx buffer on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                return;
            }

            // Purge port buffers
            ftStatus = _FTDI.Purge(FTDI.FT_PURGE.FT_PURGE_RX + FTDI.FT_PURGE.FT_PURGE_TX);
            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                DebugLog.Enqueue($"Failed to purge buffers on FTDI device with S/N {SN}. Error: {ftStatus.ToString()}");
                return;
            }

            if (numBytes > 0)
            {
                if (PediatricSensorData.DebugMode)
                {
                    DebugLog.Enqueue($"Sensor {SN}: Valid");
                }

                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Valid;
                }
                IsValid = true;
            }

        }

        private void SaveConfig()
        {
            if (pediatricSensorConfig.Equals(pediatricSensorConfigOnLoad))
            {
                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Configuration not changed - not saving");
            }
            else
            {
                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Saving configuration");

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
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Loading configuration");

            TextReader reader = null;
            try
            {
                XmlSerializer serializer = new XmlSerializer(typeof(PediatricSensorConfig));
                reader = new StreamReader(configPath);
                pediatricSensorConfig = (PediatricSensorConfig)serializer.Deserialize(reader);
            }
            catch (Exception)
            {
                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Failed to load configuration - using defaults");
                pediatricSensorConfig = new PediatricSensorConfig();
            }
            finally
            {
                if (reader != null)
                    reader.Close();
            }

            pediatricSensorConfigOnLoad = pediatricSensorConfig.GetValueCopy();
        }

        private void SendCommandsSetup(PediatricSoftConstants.SensorState correctState)
        {
            PediatricSoftConstants.SensorState currentState = correctState;

            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLockDisableAll)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDInverse);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDInverse)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdLaserCurrent)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrentModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserCurrentModulation)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorColdLaserHeat)));

            SendCommand(PediatricSoftConstants.SensorCommandBxOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultBxOffset)));

            SendCommand(PediatricSoftConstants.SensorCommandBxModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(ushort.MinValue)));

            SendCommand(PediatricSoftConstants.SensorCommandByOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultByOffset)));

            SendCommand(PediatricSoftConstants.SensorCommandByModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(ushort.MinValue)));

            SendCommand(PediatricSoftConstants.SensorCommandBzOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultBzOffset)));

            SendCommand(PediatricSoftConstants.SensorCommandBzModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(ushort.MinValue)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserModulationFrequency);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserModulationFrequency)));

            SendCommand(PediatricSoftConstants.SensorCommandDelayForLaser);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultDelayForLaser)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDLaserCurrentP);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDLaserCurrentP)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDLaserCurrentI);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.LaserCurrentKI)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDLaserHeaterI);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.LaserHeatKI)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDCellHeaterI);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.CellHeatKI)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDByP);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDByP)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDBzP);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDBzP)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDByI);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultPIDByI)));

            SendCommand(PediatricSoftConstants.SensorCommandPIDBzI);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.BzKI)));

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

            SendCommand(PediatricSoftConstants.SensorCommandLED);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLEDOff)));

            SendCommand(PediatricSoftConstants.SensorCommandLaserHeatLimit);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorMaxLaserHeat)));

            SendCommand(PediatricSoftConstants.SensorCommandCellHeatLimit);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.MaxCellHeat)));

            while (commandQueue.TryPeek(out string dummy) && currentState == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            CalibrationBzDemod = 0;
            coldSensorADCValue = 0;

            lock (stateLock)
            {
                if (State == PediatricSoftConstants.SensorState.Setup)
                {
                    State++;
                }
            }
        }

        private void SendCommandsStartLock(PediatricSoftConstants.SensorState correctState)
        {
            Stopwatch stopwatch = new Stopwatch();
            PediatricSoftConstants.SensorState currentState = correctState;

            // Turn on the LED
            SendCommand(PediatricSoftConstants.SensorCommandLED);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLEDRed)));

            // Turn on the Laser
            SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.LaserCurrent)));

            // Set the laser heater to the default value
            SendCommand(PediatricSoftConstants.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultLaserHeat)));

            // Set the cell heater to the default value
            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.DefaultCellHeat)));

            while (commandQueue.TryPeek(out string dummy) && currentState == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            // Wait a bit
            stopwatch.Restart();
            while (stopwatch.ElapsedMilliseconds < PediatricSoftConstants.CellHeatInitialTime &&
                   currentState == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }
            stopwatch.Stop();

            // If we failed - return
            if (currentState != correctState)
            {
                DebugLog.Enqueue($"Sensor {SN}: Lock was aborted or something failed. Returning.");
                return;
            }

            // Record the ADC value
            lock (dataLock)
            {
                coldSensorADCValue = dataPoints.Select(x => x.ADC)
                                               .Average();
            }

            // Here we check if the laser is working as expected
            if (coldSensorADCValue < PediatricSoftConstants.SensorADCColdValueLowGainMinVolts ||
                coldSensorADCValue > PediatricSoftConstants.SensorADCColdValueLowGainMaxVolts)
            {
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.SoftFail;
                }
                DebugLog.Enqueue($"Sensor {SN} failed: ADC value out of range ({coldSensorADCValue} V)");
                return;
            }

            // Turn the Z-Coil to max current
            SendCommand(PediatricSoftConstants.SensorCommandBzOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(ushort.MaxValue)));

            // Set the cell heater to the max value
            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.MaxCellHeat)));

            while (commandQueue.TryPeek(out string dummy) && currentState == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            lock (stateLock)
            {
                if (currentState == correctState)
                {
                    State++;
                }
            }

        }

        private void SendCommandsLaserLockStep(PediatricSoftConstants.SensorState correctState)
        {
            Stopwatch stopwatch = new Stopwatch();
            PediatricSoftConstants.SensorState currentState = correctState;

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
                if (PediatricSensorData.DebugMode)
                {
                    DebugLog.Enqueue($"Sensor {SN}: Laser lock step cycle: {i}");
                }

                while (laserHeat < PediatricSoftConstants.SensorMaxLaserHeat)
                {
                    while (currentState == correctState)
                    {
                        lock (dataLock)
                        {
                            if (dataUpdated)
                            {
                                break;
                            }
                        }
                        Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (stateLock)
                        {
                            currentState = State;
                        }
                    }

                    // If we failed - return
                    if (currentState != correctState)
                    {
                        DebugLog.Enqueue($"Sensor {SN}: Lock was aborted or something failed. Returning.");
                        return;
                    }

                    transmission = lastDataPoint.ADC / coldSensorADCValue;

                    if (transmission < PediatricSoftConstants.SensorTargetLaserTransmissionStep)
                    {
                        if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Step cycle - found Rb resonance");
                        if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Transmission on resonance: {transmission}");

                        lock (stateLock)
                        {
                            if (State == correctState)
                            {
                                State++;
                            }
                        }
                        return;
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
                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)));

                // Wait a bit
                stopwatch.Restart();
                while (stopwatch.ElapsedMilliseconds < PediatricSoftConstants.LaserLockWaitBetweenIterations &&
                       currentState == correctState)
                {
                    Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }
                stopwatch.Stop();

            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State = PediatricSoftConstants.SensorState.SoftFail;
                }
            }
            DebugLog.Enqueue($"Sensor {SN}: Laser lock failed. Giving up");

        }

        private void SendCommandsLaserLockPID(PediatricSoftConstants.SensorState correctState)
        {
            PediatricSoftConstants.SensorState currentState = correctState;

            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndADCGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataStreamingOnGainHigh)));

            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLockOpticalResonance)));

            while (commandQueue.TryPeek(out string dummy) && currentState == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            coldSensorADCValue = coldSensorADCValue * 50 / 15.5;

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Lock was aborted or something failed. Returning.");
                }
            }
        }

        private void SendCommandsCellHeatLock(PediatricSoftConstants.SensorState correctState)
        {
            PediatricSoftConstants.SensorState currentState = correctState;

            double targetADCValue = coldSensorADCValue * PediatricSoftConstants.SensorTargetLaserTransmissionOffMagneticResonance;
            if (targetADCValue > PediatricSoftConstants.SensorADCColdValueLowGainMaxVolts)
            {
                targetADCValue = PediatricSoftConstants.SensorADCColdValueLowGainMaxVolts;
            }
            ushort cellHeatLockPoint = (ushort)(ushort.MaxValue * (targetADCValue / 5));

            SendCommand(PediatricSoftConstants.SensorCommandCellHeatLockPoint);
            SendCommand(String.Concat("#", UInt16ToStringBE(cellHeatLockPoint)));

            // This is to prevent the heat lock from oscillating
            SendCommand(PediatricSoftConstants.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.DefaultCellHeat)));

            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLockOpticalResonance |
                                                            PediatricSoftConstants.SensorLockCellTemperature)));

            while (commandQueue.TryPeek(out string dummy) && currentState == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsStabilizeCellHeat(PediatricSoftConstants.SensorState correctState)
        {
            PediatricSoftConstants.SensorState currentState = correctState;
            Stopwatch stopwatch = new Stopwatch();

            int arraySize = PediatricSoftConstants.StabilizeCellHeatTimeWindow / PediatricSoftConstants.StabilizeCellHeatMeasurementInterval;
            double[] transmissionArray = new double[arraySize];
            int arrayIndex = 0;

            double minTransmission = PediatricSoftConstants.SensorTargetLaserTransmissionOffMagneticResonance - PediatricSoftConstants.StabilizeCellHeatTolerance;
            double maxTransmission = PediatricSoftConstants.SensorTargetLaserTransmissionOffMagneticResonance + PediatricSoftConstants.StabilizeCellHeatTolerance;

            stopwatch.Start();

            while (currentState == correctState)
            {
                lock (dataLock)
                {
                    transmissionArray[arrayIndex] = lastDataPoint.ADC / coldSensorADCValue;
                }

                if ((transmissionArray.Min() > minTransmission) && (transmissionArray.Max() < maxTransmission))
                {
                    break;
                }

                arrayIndex++;
                if (arrayIndex == arraySize)
                {
                    arrayIndex = 0;
                }

                Thread.Sleep(PediatricSoftConstants.StabilizeCellHeatMeasurementInterval);

                if (stopwatch.ElapsedMilliseconds > PediatricSoftConstants.StabilizeCellHeatFailAfter)
                {
                    lock (stateLock)
                    {
                        State = PediatricSoftConstants.SensorState.SoftFail;
                    }
                    DebugLog.Enqueue($"Sensor {SN}: Failed to stabilize cell heat");
                }

                lock (stateLock)
                {
                    currentState = State;
                }
            }

            stopwatch.Stop();

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsHoldCurrentCellHeat(PediatricSoftConstants.SensorState correctState)
        {

            currentCellHeat = SendCommandsHoldCurrentParameter(correctState, PediatricSoftConstants.SensorCommandCellHeat, PediatricSoftConstants.SensorLockCellTemperature);

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsZeroFieldsYZ(PediatricSoftConstants.SensorState correctState)
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Begin field zeroing");

            ushort fieldStep = ushort.MaxValue / PediatricSoftConstants.NumberOfFieldZeroingIntervalsOneAxis - 1;
            int maxADCRAWValue = int.MinValue;
            int currentADCRAWValue = int.MinValue;

            // Reset fields and stuff

            CalibrationBzDemod = 0;

            SendCommand(PediatricSoftConstants.SensorCommandLED);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLEDRed)));

            SendCommand(PediatricSoftConstants.SensorCommandBxOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultBxOffset)));

            SendCommand(PediatricSoftConstants.SensorCommandBxModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(ushort.MinValue)));

            SendCommand(PediatricSoftConstants.SensorCommandByModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(ushort.MinValue)));

            SendCommand(PediatricSoftConstants.SensorCommandBzModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(ushort.MinValue)));

            for (ushort yField = ushort.MinValue; yField < ushort.MaxValue; yField = UShortSafeInc(yField, fieldStep, ushort.MaxValue))
            {
                for (ushort zField = ushort.MinValue; zField < ushort.MaxValue; zField = UShortSafeInc(zField, fieldStep, ushort.MaxValue))
                {
                    SendCommand(PediatricSoftConstants.SensorCommandByOffset);
                    SendCommand(String.Concat("#", UInt16ToStringBE(yField)));

                    SendCommand(PediatricSoftConstants.SensorCommandBzOffset);
                    SendCommand(String.Concat("#", UInt16ToStringBE(zField)));

                    while (commandQueue.TryPeek(out string dummy))
                    {
                        Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        if (State != correctState)
                        {
                            DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
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
                            if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                            return;
                        }
                    }

                    currentADCRAWValue = lastDataPoint.ADCRAW;

                    if (currentADCRAWValue > maxADCRAWValue)
                    {
                        maxADCRAWValue = currentADCRAWValue;

                        zeroBy = yField;
                        zeroBz = zField;

                    }

                }

            }

            SendCommand(PediatricSoftConstants.SensorCommandByOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(zeroBy)));

            SendCommand(PediatricSoftConstants.SensorCommandBzOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(zeroBz)));

            SendCommand(PediatricSoftConstants.SensorCommandByModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDefaultByModulation)));

            SendCommand(PediatricSoftConstants.SensorCommandBzModulation);
            SendCommand(String.Concat("#", UInt16ToStringBE(pediatricSensorConfig.BzModulation)));

            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLockOpticalResonance |
                                                            PediatricSoftConstants.SensorLockBz |
                                                            PediatricSoftConstants.SensorLockBy)));

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            Thread.Sleep(PediatricSoftConstants.TransmissionAveragingTime);

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

            if (PediatricSensorData.DebugMode)
            {
                DebugLog.Enqueue($"Sensor {SN}: Field zeroing done");
            }
        }

        private void SendCommandsHoldCurrentBy(PediatricSoftConstants.SensorState correctState)
        {

            zeroBy = SendCommandsHoldCurrentParameter(correctState, PediatricSoftConstants.SensorCommandByOffset, PediatricSoftConstants.SensorLockBy);

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsHoldCurrentBz(PediatricSoftConstants.SensorState correctState)
        {

            zeroBz = SendCommandsHoldCurrentParameter(correctState, PediatricSoftConstants.SensorCommandBzOffset, PediatricSoftConstants.SensorLockBz);

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsZeroFieldX(PediatricSoftConstants.SensorState correctState)
        {

            ushort fieldStep = ushort.MaxValue / PediatricSoftConstants.NumberOfFieldZeroingIntervalsOneAxis - 1;

            double bestCalibrationBzDemod = double.PositiveInfinity;

            for (ushort currentBx = ushort.MinValue; currentBx < ushort.MaxValue; currentBx = UShortSafeInc(currentBx, fieldStep, ushort.MaxValue))
            {
                SendCommand(PediatricSoftConstants.SensorCommandBxOffset);
                SendCommand(String.Concat("#", UInt16ToStringBE(currentBx)));

                CalibrationBzDemod = CalibrateMagnetometer(correctState);

                if (Math.Abs(CalibrationBzDemod) < Math.Abs(bestCalibrationBzDemod))
                {
                    bestCalibrationBzDemod = CalibrationBzDemod;
                    zeroBx = currentBx;

                }

            }

            SendCommand(PediatricSoftConstants.SensorCommandBxOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(zeroBx)));

            CalibrationBzDemod = bestCalibrationBzDemod;

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

            if (PediatricSensorData.DebugMode)
            {
                DebugLog.Enqueue($"Sensor {SN}: Field zeroing done");
            }
        }

        private void SendCommandsHoldCurrentTransmission(PediatricSoftConstants.SensorState correctState)
        {

            Thread.Sleep(PediatricSoftConstants.TransmissionAveragingTime);

            double targetADCValue = 0;

            // Check the transmission
            lock (dataLock)
            {
                targetADCValue = dataPoints.Select(x => x.ADC)
                                         .Skip(dataPoints.Length - PediatricSoftConstants.TransmissionAveragingTime)
                                         .Average();
            }

            if (targetADCValue < coldSensorADCValue * PediatricSoftConstants.SensorMinLaserTransmissionRun)
            {
                targetADCValue = coldSensorADCValue * PediatricSoftConstants.SensorMinLaserTransmissionRun;
            }

            ushort cellHeatLockPoint = (ushort)(ushort.MaxValue * (targetADCValue / 5));

            SendCommand(PediatricSoftConstants.SensorCommandCellHeatLockPoint);
            SendCommand(String.Concat("#", UInt16ToStringBE(cellHeatLockPoint)));

            SendCommand(PediatricSoftConstants.SensorCommandLock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLockOpticalResonance |
                                                            PediatricSoftConstants.SensorLockBz |
                                                            PediatricSoftConstants.SensorLockBy |
                                                            PediatricSoftConstants.SensorLockCellTemperature)));

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsSwitchMagnetometerMode(PediatricSoftConstants.SensorState correctState)
        {
            SendCommand(PediatricSoftConstants.SensorCommandLED);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLEDRed)));

            if (PediatricSensorData.MagnetometerMode == PediatricSoftConstants.MagnetometerMode.OpenLoop)
            {
                zeroBy = SendCommandsHoldCurrentParameter(correctState, PediatricSoftConstants.SensorCommandByOffset, PediatricSoftConstants.SensorLockBy);
                zeroBz = SendCommandsHoldCurrentParameter(correctState, PediatricSoftConstants.SensorCommandBzOffset, PediatricSoftConstants.SensorLockBz);
            }
            else
            {
                SendCommand(PediatricSoftConstants.SensorCommandLock);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLockOpticalResonance |
                                                                PediatricSoftConstants.SensorLockBz |
                                                                PediatricSoftConstants.SensorLockBy |
                                                                PediatricSoftConstants.SensorLockCellTemperature)));
            }

            SendCommand(PediatricSoftConstants.SensorCommandLED);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLEDBlue)));

            while (commandQueue.TryPeek(out string dummy))
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                if (State != correctState)
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                    return;
                }
            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }
        }

        private void SendCommandsStartDataSave(PediatricSoftConstants.SensorState correctState)
        {

            if (PediatricSensorData.SaveDataEnabled)
            {
                lock (dataLock)
                {
                    filePath = System.IO.Path.Combine(PediatricSensorData.SaveFolderCurrentRun, SN);
                    filePath += ".txt";

                    dataSaveBuffer.Clear();

                    dataSaveEnable = PediatricSensorData.SaveDataEnabled;
                    dataSaveRAW = PediatricSensorData.SaveRAWValues;
                }
            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsSyncTimers(PediatricSoftConstants.SensorState correctState)
        {
            PediatricSoftConstants.SensorState currentState = correctState;

            double startTime = 0;
            double currentTime = double.MaxValue;

            // If master card is present - sync timers
            if (PediatricSensorData.IsMasterCardPresent)
            {

                lock (dataLock)
                {
                    startTime = lastDataPoint.Time;
                }

                // If we are the master board - send out the sync pulse
                if (IsMasterCard)
                {
                    SendCommand(PediatricSoftConstants.SensorCommandTriggers);
                    SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorSyncTimers)));

                    while (commandQueue.TryPeek(out string dummy) && currentState == correctState)
                    {
                        Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                        lock (stateLock)
                        {
                            currentState = State;
                        }
                    }
                }

                // Wait for the clocks to reset
                while (currentTime > startTime && currentState == correctState)
                {
                    Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                    lock (dataLock)
                    {
                        currentTime = lastDataPoint.Time;
                    }
                    lock (stateLock)
                    {
                        currentState = State;
                    }
                }

            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsStopDataSave(PediatricSoftConstants.SensorState correctState)
        {

            if (dataSaveEnable)
            {
                lock (dataLock)
                {

                    File.AppendAllLines(filePath, dataSaveBuffer);
                    dataSaveBuffer.Clear();
                    dataSaveEnable = false;
                }
            }

            lock (stateLock)
            {
                if (State == correctState)
                {
                    State++;
                }
                else
                {
                    DebugLog.Enqueue($"Sensor {SN}: Procedure was aborted or something failed. Returning.");
                }
            }

        }

        private void SendCommandsShutDown(PediatricSoftConstants.SensorState correctState)
        {
            PediatricSoftConstants.SensorState currentState = correctState;

            SendCommandsSetup(correctState);

            // Disable streaming
            SendCommand(PediatricSoftConstants.SensorCommandDigitalDataStreamingAndADCGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorDigitalDataStreamingOffGainLow)));

            // Turn off LEDs if shutting down
            if (correctState == PediatricSoftConstants.SensorState.ShutDownRequested)
            {
                SendCommand(PediatricSoftConstants.SensorCommandLED);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLEDOff)));
            }

            // Blink LEDs if failing
            if (correctState == PediatricSoftConstants.SensorState.SoftFail)
            {
                SendCommand(PediatricSoftConstants.SensorCommandLED);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSoftConstants.SensorLEDBlueBlinkRedBlink)));
            }

            while (commandQueue.TryPeek(out string dummy) && currentState == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
                lock (stateLock)
                {
                    currentState = State;
                }
            }

            lock (stateLock)
            {
                if (currentState == PediatricSoftConstants.SensorState.SoftFail)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                }
                else
                {
                    if (currentState == PediatricSoftConstants.SensorState.ShutDownRequested)
                    {
                        State = PediatricSoftConstants.SensorState.ShutDownComplete;
                    }
                    else
                    {
                        DebugLog.Enqueue($"Sensor {SN}: We are in the shutdown procedure, but we shouldn't be here. Failing..");
                        State = PediatricSoftConstants.SensorState.Failed;
                    }
                }
            }

        }

        private ushort SendCommandsHoldCurrentParameter(PediatricSoftConstants.SensorState correctState, string command, ushort lockBits)
        {

            // We first query the current lock bits
            lock (dataLock)
            {
                SendCommand(PediatricSoftConstants.SensorCommandLock);
                SendCommand("?");
                infoRequested = true;
            }

            while (infoRequested && State == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
            }

            if (ushort.TryParse(requestedInfoString, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort currentLockBits) && State == correctState)
            {
                // Success
            }
            else
            {
                // We failed to get the current parameter
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                    return 0;
                }
            }

            // Now that we have the lock bits - query the parameter
            lock (dataLock)
            {
                SendCommand(command);
                SendCommand("?");
                infoRequested = true;
            }

            while (infoRequested && State == correctState)
            {
                Thread.Sleep(PediatricSoftConstants.StateHandlerSleepTime);
            }

            if (ushort.TryParse(requestedInfoString, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort currentParameter))
            {
                // Success
                // Turn off the lock but keep the value

                SendCommand(command);
                SendCommand(String.Concat("#", UInt16ToStringBE(currentParameter)));

                // Use XOR to disable the appropriate lock
                SendCommand(PediatricSoftConstants.SensorCommandLock);
                SendCommand(String.Concat("#", UInt16ToStringBE((ushort)(currentLockBits ^ lockBits))));

                while (commandQueue.TryPeek(out string dummy) && State == correctState)
                {
                    Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
                }
            }
            else
            {
                // We failed to get the current parameter
                lock (stateLock)
                {
                    State = PediatricSoftConstants.SensorState.Failed;
                    return 0;
                }
            }

            if (State == correctState)
            {
                return currentParameter;
            }
            else
            {
                return 0;
            }

        }

        private double CalibrateMagnetometer(PediatricSoftConstants.SensorState correctState)
        {
            double calibrationBzDemod = 0;

            ushort plusField = UShortSafeInc(zeroBz, PediatricSoftConstants.SensorCoilCalibrationStepHex, ushort.MaxValue);
            ushort minusField = UShortSafeDec(zeroBz, PediatricSoftConstants.SensorCoilCalibrationStepHex, ushort.MinValue);

            double plusValue = 0;
            double minusValue = 0;

            SendCommand(PediatricSoftConstants.SensorCommandBzOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(plusField)));

            while (commandQueue.TryPeek(out string dummy) && State == correctState)
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
            }

            Thread.Sleep(PediatricSoftConstants.TransmissionAveragingTime);

            lock (dataLock)
            {
                plusValue = dataPoints.Select(x => x.BzDemodRAW)
                                         .Skip(dataPoints.Length - PediatricSoftConstants.TransmissionAveragingTime)
                                         .Average();
            }

            SendCommand(String.Concat("#", UInt16ToStringBE(minusField)));

            while (commandQueue.TryPeek(out string dummy) && State == correctState)
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
            }

            Thread.Sleep(PediatricSoftConstants.TransmissionAveragingTime);

            lock (dataLock)
            {
                minusValue = dataPoints.Select(x => x.BzDemodRAW)
                                         .Skip(dataPoints.Length - PediatricSoftConstants.TransmissionAveragingTime)
                                         .Average();
            }

            SendCommand(String.Concat("#", UInt16ToStringBE(zeroBz)));

            while (commandQueue.TryPeek(out string dummy) && State == correctState)
            {
                Thread.Sleep((int)PediatricSoftConstants.SerialPortReadTimeout);
            }

            calibrationBzDemod = PediatricSoftConstants.SensorZCoilCalibrationTeslaPerHex / ((plusValue - minusValue) / (plusField - minusField));

            if (PediatricSensorData.DebugMode)
            {
                DebugLog.Enqueue($"Sensor {SN}: ZDemod calibration {calibrationBzDemod}");
            }

            return calibrationBzDemod;

        }

        public void Dispose()
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Dispose() was called");

            if (IsValid)
            {
                SaveConfig();
            }

            lock (stateLock)
            {
                if (State > PediatricSoftConstants.SensorState.Init && State < PediatricSoftConstants.SensorState.ShutDownRequested)
                {
                    State = PediatricSoftConstants.SensorState.ShutDownRequested;
                }
            }

            if (uiUpdateTimer != null)
            {
                if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Sensor {SN}: Disposing the UI timer");
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
            dataFFTWindow = MathNet.Numerics.Window.HannPeriodic(PediatricSoftConstants.FFTLength);
            dataFFTWindowAmplitudeFactor = PediatricSoftConstants.FFTLength / dataFFTWindow.Sum();
            dataFFTWindowNoisePowerBandwidth = 1.5; // Hann Window

            double[] frequencyScale = Fourier.FrequencyScale(PediatricSoftConstants.FFTLength, PediatricSoftConstants.DataSampleRate);
            dataFFTFrequencyStep = frequencyScale[1];

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