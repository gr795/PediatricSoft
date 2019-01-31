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

namespace PediatricSoft
{
    public sealed class PediatricSensor : INotifyPropertyChanged, IDisposable
    {

        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        private SerialPort _SerialPort;
        private Task streamingTask;
        private Task processingTask;
        private Task dataSaveTask;
        private Task stateTask;
        private System.Timers.Timer uiUpdateTimer;
        private Random rnd = new Random();
        private CancellationTokenSource streamCancellationTokenSource;
        private CancellationTokenSource fileSaveCancellationTokenSource;

        private byte state = PediatricSensorData.SensorStateInit;
        private readonly Object stateLock = new Object();
        private int sensorADCColdValueRaw;
        private ushort laserHeat = PediatricSensorData.SensorMinLaserHeat;
        private bool foundResonanceWiggle = false;

        private readonly byte[] buffer = new byte[PediatricSensorData.ProcessingBufferSize];
        private int bufferIndex = 0;
        private readonly Object bufferLock = new Object();

        private bool dataSaveEnable = false;
        private List<string> dataSaveBuffer = new List<string>();
        private readonly Object dataSaveLock = new Object();

        private readonly Object lastValueLock = new Object();


        private Queue<int> dataTimeRaw = new Queue<int>(PediatricSensorData.DataQueueLength);
        private Queue<int> dataValueRaw = new Queue<int>(PediatricSensorData.DataQueueLength);

        public ChartValues<ObservableValue> _ChartValues = new ChartValues<ObservableValue>();
        public bool ShouldBePlotted { get; set; } = false;
        public int LastValue { get; private set; } = 0;
        public int LastTime { get; private set; } = 0;
        public double Voltage { get { return (double)LastValue * 5 / 125 / 16777215; } }

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
            OnPropertyChanged("Voltage");
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
                        _SerialPort.WriteLine("?");
                        Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                        int bytesToRead = _SerialPort.BytesToRead;
                        if (bytesToRead > 0)
                        {
                            _SerialPort.DiscardInBuffer();
                            state = PediatricSensorData.SensorStateValid;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

                            streamCancellationTokenSource = new CancellationTokenSource();
                            streamingTask = Task.Run(() => StreamData(streamCancellationTokenSource.Token));
                            processingTask = Task.Run(() => ProcessData());
                            stateTask = Task.Run(() => StateHandler());

                            uiUpdateTimer = new System.Timers.Timer(PediatricSensorData.UIUpdateInterval);
                            uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
                            uiUpdateTimer.Enabled = true;
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

        public void Lock()
        {

            // We enable data streaming and start the state handler
            if (state == PediatricSensorData.SensorStateValid)
            {
                if (_SerialPort != null)
                    if (_SerialPort.IsOpen)
                    {
                        try
                        {
                            _SerialPort.WriteLine("@20");
                            _SerialPort.WriteLine("#2");
                        }
                        catch (Exception e) { Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: {e.Message}"); }

                    }

            }

        }

        public void Start()
        {
            if (PediatricSensorData.DebugMode && state == PediatricSensorData.SensorStateValid)
            {
                state = PediatricSensorData.SensorStateIdle;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }

            if (state == PediatricSensorData.SensorStateIdle)
            {
                state++;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }
        }

        public void Stop()
        {
            if (state == PediatricSensorData.SensorStateRun)
            {

                state++;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");


                // if (PediatricSensorData.SaveDataEnabled && (file != null)) file.Dispose();

            }
        }

        private void StreamData(CancellationToken cancellationToken)
        {
            Stream stream = _SerialPort.BaseStream;
            byte[] byteArrayIn = new byte[PediatricSensorData.SerialPortStreamBlockSize];
            int bytesRead = 0;

            while (!cancellationToken.IsCancellationRequested)
            {
                try
                {
                    bytesRead = stream.Read(byteArrayIn, 0, PediatricSensorData.SerialPortStreamBlockSize);
                }
                catch (UnauthorizedAccessException e)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: {e.Message}");
                    streamCancellationTokenSource.Cancel();
                    state = PediatricSensorData.SensorStateFailed;
                    OnPropertyChanged("State");
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                }
                catch (TimeoutException)
                {

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
                Thread.Sleep(rnd.Next(PediatricSensorData.SerialPortStreamSleepMin, PediatricSensorData.SerialPortStreamSleepMax));
            }

        }



        private void ProcessData()
        {
            byte[] data = new byte[PediatricSensorData.DataBlockSize];
            int dataIndex = 0;

            byte[] info = new byte[PediatricSensorData.InfoBlockSize];
            int infoIndex = 0;

            byte[] localBuffer = new byte[PediatricSensorData.ProcessingBufferSize];
            int localBufferIndex = 0;

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
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Info message: {System.Text.Encoding.ASCII.GetString(info)}");
                            PediatricSensorData.CommandHistory = String.Concat(System.Text.Encoding.ASCII.GetString(info), "\n", PediatricSensorData.CommandHistory);
                            inInfoFrame = false;
                            infoIndex = 0;
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
                                plotCounter++;
                                if (plotCounter == plotCounterMax)
                                {
                                    _ChartValues.Add(new ObservableValue(LastValue));
                                    if (_ChartValues.Count > PediatricSensorData.PlotQueueLength) _ChartValues.RemoveAt(0);
                                    plotCounter = 0;
                                }
                            }

                            lock (dataSaveLock)
                            {
                                if (dataSaveEnable)
                                    dataSaveBuffer.Add(String.Concat(Convert.ToString(LastTime), "\t", Convert.ToString(LastValue)));
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

            string filePath = System.IO.Path.Combine(PediatricSensorData.dataFolder, SN);
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

            SendCommand(PediatricSensorData.SensorCommandLaserlock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserlockDisable)));

            SendCommand(PediatricSensorData.SensorCommandLaserCurrentMod);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserCurrentModValue)));

            SendCommand(PediatricSensorData.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleLaserCurrent)));

            SendCommand(PediatricSensorData.SensorCommandLaserHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleLaserHeat)));

            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleCellHeat)));

            SendCommand(PediatricSensorData.SensorCommandFieldXOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleFieldXOffset)));

            SendCommand(PediatricSensorData.SensorCommandFieldXAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleFieldXAmplitude)));

            SendCommand(PediatricSensorData.SensorCommandFieldYOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleFieldYOffset)));

            SendCommand(PediatricSensorData.SensorCommandFieldYAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleFieldYAmplitude)));

            SendCommand(PediatricSensorData.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleFieldZOffset)));

            SendCommand(PediatricSensorData.SensorCommandFieldZAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorIdleFieldZAmplitude)));

        }

        private void SendCommandsLaserLockSweep()
        {

            bool foundResonanceSweep = false;
            int numberOfLaserLockSweepCycles = 0;
            int[] data = new int[PediatricSensorData.DataQueueLength];
            double transmission = 0;

            // Turn on the laser
            SendCommand(PediatricSensorData.SensorCommandLaserCurrent);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDefaultLaserCurrent)));

            // Wait a bit and record the ADC value
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerADCColdDelay} ms");
            Thread.Sleep(PediatricSensorData.StateHandlerADCColdDelay);
            sensorADCColdValueRaw = LastValue;

            // Here we check if we received non-zero value.
            // If it is zero (default) - something is wrong and we fail.
            if (sensorADCColdValueRaw == 0)
            {
                SendCommandsColdSensor();
                state = PediatricSensorData.SensorStateFailed;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }
            else
            {
                // Set the cell heater to the max value, wait for the cell to warm up
                SendCommand(PediatricSensorData.SensorCommandCellHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorMaxCellHeat)));
            }

            // Laser heat sweep cycle. Here we look for the Rb resonance.
            while (!foundResonanceSweep && state == PediatricSensorData.SensorStateLaserLockSweep && numberOfLaserLockSweepCycles < PediatricSensorData.MaxNumberOfLaserLockSweepCycles)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Laser lock sweep cycle: {numberOfLaserLockSweepCycles}");

                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerCellHeatInitialTime} ms");
                Thread.Sleep(PediatricSensorData.StateHandlerCellHeatInitialTime);

                // Set the laser heater to the max value, wait a bit
                SendCommand(PediatricSensorData.SensorCommandLaserHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorMaxLaserHeat)));
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Waiting for {PediatricSensorData.StateHandlerLaserHeatSweepTime} ms");
                Thread.Sleep(PediatricSensorData.StateHandlerLaserHeatSweepTime);

                // See if the threshold was reached
                data = dataValueRaw.ToArray();
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
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }

        }

        private void SendCommandsLaserLockStep()
        {

            bool foundResonanceStep = false;
            int numberOfLaserLockStepCycles = 0;
            double transmission = 0;

            Thread.Sleep(PediatricSensorData.SensorLaserHeatStepCycleDelay);

            while (!foundResonanceStep && state == PediatricSensorData.SensorStateLaserLockStep && numberOfLaserLockStepCycles < PediatricSensorData.MaxNumberOfLaserLockStepCycles)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Laser lock step cycle: {numberOfLaserLockStepCycles}");

                laserHeat = PediatricSensorData.SensorMinLaserHeat;
                SendCommand(PediatricSensorData.SensorCommandLaserHeat, PediatricSensorData.IsLaserLockDebugEnabled);
                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeat)), PediatricSensorData.IsLaserLockDebugEnabled);

                while (!foundResonanceStep && state == PediatricSensorData.SensorStateLaserLockStep && laserHeat < PediatricSensorData.SensorMaxLaserHeat)
                {
                    Thread.Sleep(PediatricSensorData.SensorLaserHeatStepSleepTime);

                    transmission = (double)LastValue / sensorADCColdValueRaw;
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
                    }

                }


                numberOfLaserLockStepCycles++;
            }

            if (!foundResonanceStep)
            {
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: We saw Rb resonance, but the step cycle failed. Giving up");
                SendCommandsColdSensor();
                state = PediatricSensorData.SensorStateFailed;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }


        }

        private void SendCommandsLaserLockWiggle()
        {

            int wiggleIterationMax = PediatricSensorData.SensorLaserHeatWiggleCycleLength / 2 / PediatricSensorData.SensorLaserHeatWiggleSleepTime;
            int sensorADCPlus = 0;
            int sensorADCMinus = 0; ;
            ushort laserHeatPlus = 0;
            ushort laserHeatMinus = 0;

            double transmission = 0;

            Thread.Sleep(PediatricSensorData.SensorLaserHeatWiggleCycleDelay);
            SendCommand(PediatricSensorData.SensorCommandLaserHeat, PediatricSensorData.IsLaserLockDebugEnabled);
            //sensorADCLast = LastValue;

            for (int wiggleIteration = 0; wiggleIteration < wiggleIterationMax; wiggleIteration++)
            {
                laserHeatPlus = UShortSafeInc(laserHeat, PediatricSensorData.SensorLaserHeatWiggleStep, PediatricSensorData.SensorMaxLaserHeat);
                laserHeatMinus = UShortSafeDec(laserHeat, PediatricSensorData.SensorLaserHeatWiggleStep, PediatricSensorData.SensorMinLaserHeat);

                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeatPlus)), PediatricSensorData.IsLaserLockDebugEnabled);
                Thread.Sleep(PediatricSensorData.SensorLaserHeatWiggleSleepTime);
                sensorADCPlus = LastValue;

                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeatMinus)), PediatricSensorData.IsLaserLockDebugEnabled);
                Thread.Sleep(PediatricSensorData.SensorLaserHeatWiggleSleepTime);
                sensorADCMinus = LastValue;

                if (sensorADCPlus < sensorADCMinus)
                    laserHeat = laserHeatPlus;
                if (sensorADCPlus > sensorADCMinus)
                    laserHeat = laserHeatMinus;

                transmission = (double)sensorADCMinus / sensorADCColdValueRaw;
                if (transmission > PediatricSensorData.SensorTargetLaserTransmissionWiggle) break;
            }

            if (transmission < PediatricSensorData.SensorTargetLaserTransmissionWiggle)
                foundResonanceWiggle = true;

        }

        private void SendCommandsLaserLockPID()
        {
            SendCommand(PediatricSensorData.SensorCommandLaserlock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserlockEnable)));
        }

        private void SendCommandsStabilizeCellHeat()
        {
            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorRunCellHeat)));
        }

        private void SendCommandsZeroOneAxis()
        {
            int sensorADCCurrent = 0;
            int sensorADCMax = 0;

            ushort currentField = ushort.MinValue;
            ushort zeroField = ushort.MinValue;

            while (currentField < ushort.MaxValue)
            {
                sensorADCCurrent = LastValue;
                if (sensorADCCurrent > sensorADCMax)
                {
                    sensorADCMax = sensorADCCurrent;
                    zeroField = currentField;
                }

                SendCommand(String.Concat("#", UInt16ToStringBE(currentField)), PediatricSensorData.IsLaserLockDebugEnabled);
                //Thread.Sleep(PediatricSensorData.SensorFieldStepDelay);
                currentField = UShortSafeInc(currentField, PediatricSensorData.SensorFieldStep, ushort.MaxValue);
            }
            SendCommand(String.Concat("#", UInt16ToStringBE(zeroField)), PediatricSensorData.IsLaserLockDebugEnabled);

        }

        public void SendCommandsZeroFields()
        {

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Begin field zeroing");

            // Z-field
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Zeroing Z-field");
            SendCommand(PediatricSensorData.SensorCommandFieldZOffset, PediatricSensorData.IsLaserLockDebugEnabled);
            SendCommandsZeroOneAxis();

            // Y-field
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Zeroing Y-field");
            SendCommand(PediatricSensorData.SensorCommandFieldYOffset, PediatricSensorData.IsLaserLockDebugEnabled);
            SendCommandsZeroOneAxis();

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Field zeroing done");

        }

        private void StateHandler()
        {

            while (state > PediatricSensorData.SensorStateInit && state < PediatricSensorData.SensorStateFailed)
            {
                switch (state)
                {

                    case PediatricSensorData.SensorStateValid:
                        break;

                    case PediatricSensorData.SensorStateMakeCold:

                        SendCommandsColdSensor();
                        if (state == PediatricSensorData.SensorStateValid)
                        {
                            state++;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateCold:

                        state++;
                        OnPropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateLockStart:

                        if (state == PediatricSensorData.SensorStateLockStart)
                        {
                            state++;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLockSweep:

                        SendCommandsLaserLockSweep();
                        if (state == PediatricSensorData.SensorStateLaserLockSweep)
                        {
                            state++;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLockStep:

                        SendCommandsLaserLockStep();
                        if (state == PediatricSensorData.SensorStateLaserLockStep)
                        {
                            state++;
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLockWiggle:

                        SendCommandsLaserLockWiggle();
                        if (state == PediatricSensorData.SensorStateLaserLockWiggle)
                        {
                            if (foundResonanceWiggle)
                            {
                                state++;
                                OnPropertyChanged("State");
                                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                            }
                            else
                            {
                                state--;
                                OnPropertyChanged("State");
                                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                            }
                        }
                        break;

                    case PediatricSensorData.SensorStateLaserLockPID:

                        SendCommandsLaserLockPID();
                        state++;
                        OnPropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateStabilizeCellHeat:

                        SendCommandsStabilizeCellHeat();
                        Thread.Sleep(60000);
                        state++;
                        OnPropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateZeroFields:

                        state++;
                        OnPropertyChanged("State");
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
                            }
                            fileSaveCancellationTokenSource = new CancellationTokenSource();
                            dataSaveTask = Task.Run(() => SaveData(fileSaveCancellationTokenSource.Token));
                        }

                        state++;
                        OnPropertyChanged("State");
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
                    if (_SerialPort != null)
                        if (_SerialPort.IsOpen)
                        {
                            try
                            {
                                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled && debug, $"Sensor {SN} on port {Port}: sending command \"{command}\"");
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
                stateTask.Wait();

            if (uiUpdateTimer != null)
            {
                uiUpdateTimer.Enabled = false;
                uiUpdateTimer.Elapsed -= OnUIUpdateTimerEvent;
                uiUpdateTimer.Dispose();
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

            if (_SerialPort != null)
            {
                PortClose();
                _SerialPort.Dispose();
            };

            IsDisposed = true;
        }

        ~PediatricSensor()
        {
            if (!IsDisposed) Dispose();
        }

    }
}