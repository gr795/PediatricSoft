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
        private bool foundResonanceWiggle = false;

        private readonly byte[] buffer = new byte[PediatricSensorData.ProcessingBufferSize];
        private int bufferIndex = 0;
        private readonly Object bufferLock = new Object();

        private bool dataSaveEnable = false;
        private bool dataSaveRAW = true;
        private List<string> dataSaveBuffer = new List<string>();
        private readonly Object dataSaveLock = new Object();

        private Queue<int> dataTimeRaw = new Queue<int>(PediatricSensorData.DataQueueLength);
        private Queue<int> dataValueRaw = new Queue<int>(PediatricSensorData.DataQueueLength);
        private Queue<int> dataValueRawRunningAvg = new Queue<int>(PediatricSensorData.DataQueueRunningAvgLength);
        private readonly Object dataLock = new Object();

        public ChartValues<ObservableValue> _ChartValues = new ChartValues<ObservableValue>();
        public bool ShouldBePlotted { get; set; } = false;
        public int LastValue { get; private set; } = 0;
        public int LastTime { get; private set; } = 0;
        public double RunningAvg { get; private set; } = 0;
        private bool wasDataUpdated = false;
        public double Voltage
        {
            get
            {
                if (state < PediatricSensorData.SensorStateIdle)
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
                        Thread.Sleep(PediatricSensorData.SerialPortReadTimeout);
                        int bytesToRead = _SerialPort.BytesToRead;
                        if (bytesToRead > 0)
                        {
                            _SerialPort.DiscardInBuffer();

                            lock (stateLock)
                            {
                                state = PediatricSensorData.SensorStateValid;
                            }
                            OnPropertyChanged("State");
                            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");

                            streamCancellationTokenSource = new CancellationTokenSource();
                            streamingTask = Task.Run(() => StreamData(streamCancellationTokenSource.Token));
                            processingTask = Task.Run(() => ProcessData());

                            stateHandlerCancellationTokenSource = new CancellationTokenSource();
                            stateTask = Task.Run(() => StateHandler(stateHandlerCancellationTokenSource.Token));

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
            lock (stateLock)
            {
                if (state == PediatricSensorData.SensorStateValid)
                {
                    state++;
                    OnPropertyChanged("State");
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                }
            }
            //while (state != PediatricSensorData.SensorStateIdle) Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
        }

        public void Start()
        {
            lock (stateLock)
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
            //while (state != PediatricSensorData.SensorStateRun) Thread.Sleep(PediatricSensorData.StateHandlerSleepTime);
        }

        public void Stop()
        {
            if (state == PediatricSensorData.SensorStateRun)
            {
                state++;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }
        }

        public void ZeroFields()
        {
            if (state == PediatricSensorData.SensorStateIdle)
            {
                state = PediatricSensorData.SensorStateZeroFields;
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }
            else
                MessageBox.Show($"Sensor {SN} on port {Port}: The sensor needs to be in state {PediatricSensorData.SensorStateIdle}");
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
                catch (TimeoutException)
                {

                }
                catch (Exception e)
                {
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Port {Port}: {e.Message}");
                    streamCancellationTokenSource.Cancel();
                    state = PediatricSensorData.SensorStateFailed;
                    OnPropertyChanged("State");
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

                            if (ShouldBePlotted)
                            {
                                plotCounter++;
                                if (plotCounter == plotCounterMax)
                                {
                                    if (state < PediatricSensorData.SensorStateIdle)
                                        _ChartValues.Add(new ObservableValue(RunningAvg * PediatricSensorData.SensorADCRawToVolts));
                                    else
                                        _ChartValues.Add(new ObservableValue(RunningAvg * sensorZDemodCalibration));
                                    if (_ChartValues.Count > PediatricSensorData.PlotQueueLength) _ChartValues.RemoveAt(0);
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
            SendCommand(PediatricSensorData.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataSelectorADC)));

            SendCommand(PediatricSensorData.SensorCommandDigitalDataStreamingAndGain);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataStreamingOnGainLow)));

            SendCommand(PediatricSensorData.SensorCommandLaserlock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserlockDisable)));

            SendCommand(PediatricSensorData.SensorCommandLaserCurrentMod);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserCurrentModValue)));

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
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDefaultLaserCurrent)));

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
                OnPropertyChanged("State");
                Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
            }
            else
            {
                // Set the cell heater to the max value, wait for the cell to warm up
                SendCommand(PediatricSensorData.SensorCommandCellHeat);
                SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorMaxCellHeat)));
            }

            SendCommand(PediatricSensorData.SensorCommandFieldZOffset);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserLockFieldZOffset)));

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
                    lock (dataLock)
                    {
                        transmission = (double)LastValue / sensorADCColdValueRaw;
                    }
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
                lock (dataLock)
                {
                    sensorADCPlus = LastValue;
                }

                SendCommand(String.Concat("#", UInt16ToStringBE(laserHeatMinus)), PediatricSensorData.IsLaserLockDebugEnabled);
                Thread.Sleep(PediatricSensorData.SensorLaserHeatWiggleSleepTime);
                lock (dataLock)
                {
                    sensorADCMinus = LastValue;
                }

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
            //SendCommand(PediatricSensorData.SensorCommandDigitalDataStreamingAndGain);
            //SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataStreamingOnGainHigh)));

            SendCommand(PediatricSensorData.SensorCommandLaserlock);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorLaserlockEnable)));
        }

        private void SendCommandsStabilizeCellHeat()
        {
            SendCommand(PediatricSensorData.SensorCommandCellHeat);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorRunCellHeat)));
        }

        private ushort SendCommandsZeroOneAxis()
        {
            double sensorADCCurrent = 0;
            double sensorADCMax = 0;

            ushort currentField = ushort.MinValue;
            ushort zeroField = ushort.MinValue;

            while (currentField < ushort.MaxValue)
            {
                while (!wasDataUpdated) Thread.Sleep(PediatricSensorData.SerialPortStreamSleepMin);
                lock (dataLock)
                {
                    sensorADCCurrent = (double)LastValue;
                }
                if (sensorADCCurrent > sensorADCMax)
                {
                    sensorADCMax = sensorADCCurrent;
                    zeroField = currentField;
                }
                SendCommand(String.Concat("#", UInt16ToStringBE(currentField)), PediatricSensorData.IsLaserLockDebugEnabled);
                lock (dataLock)
                {
                    wasDataUpdated = false;
                }
                currentField = UShortSafeInc(currentField, PediatricSensorData.SensorFieldStep, ushort.MaxValue);
            }
            SendCommand(String.Concat("#", UInt16ToStringBE(zeroField)), PediatricSensorData.IsLaserLockDebugEnabled);
            return zeroField;

        }

        private void SendCommandsZeroFields()
        {

            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Begin field zeroing");

            SendCommand(PediatricSensorData.SensorCommandDigitalDataSelector);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorDigitalDataSelectorADC)));

            SendCommand(PediatricSensorData.SensorCommandFieldZAmplitude);
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorColdFieldZAmplitude)));

            // Z-field
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Zeroing Z-field");
            SendCommand(PediatricSensorData.SensorCommandFieldZOffset, PediatricSensorData.IsLaserLockDebugEnabled);
            zeroZField = SendCommandsZeroOneAxis();

            // Y-field
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Zeroing Y-field");
            SendCommand(PediatricSensorData.SensorCommandFieldYOffset, PediatricSensorData.IsLaserLockDebugEnabled);
            zeroYField = SendCommandsZeroOneAxis();

            // X-field
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Zeroing X-field");
            SendCommand(PediatricSensorData.SensorCommandFieldYOffset, PediatricSensorData.IsLaserLockDebugEnabled);
            zeroXField = SendCommandsZeroOneAxis();

            // Z-field again
            Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Zeroing Z-field");
            SendCommand(PediatricSensorData.SensorCommandFieldZOffset, PediatricSensorData.IsLaserLockDebugEnabled);
            zeroZField = SendCommandsZeroOneAxis();

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
            SendCommand(String.Concat("#", UInt16ToStringBE(PediatricSensorData.SensorRunFieldZAmplitude)));

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

                    case PediatricSensorData.SensorStateMakeCold:

                        SendCommandsColdSensor();
                        if (state == PediatricSensorData.SensorStateMakeCold)
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

                        SendCommandsZeroFields();
                        state++;
                        OnPropertyChanged("State");
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Sensor {SN} on port {Port}: Entering state {state}");
                        break;

                    case PediatricSensorData.SensorStateCalibrateMagnetometer:

                        SendCommandsCalibrateMagnetometer();
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
                                dataSaveRAW = PediatricSensorData.SaveRAWValues;
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