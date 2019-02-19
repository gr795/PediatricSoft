using System;

namespace PediatricSoft
{
    public static class PediatricSoftConstants
    {

        // Constants
        public const bool IsDebugEnabled = true;
        public const bool IsLaserLockDebugEnabled = false;
        public const int NumberOfThreads = 128;
        public const int DataQueueLength = 5000; // number of data points to hold in memory and plot
        public const int DataQueueRunningAvgLength = 10; // number of data points for the running average
        public const int PlotQueueLength = 500;
        public const int UIUpdateInterval = 250; // Update UI every X ms
        public const string ValidIDN = "Arrow USB Blaster B";
        public const string SensorConfigFolderRelative = "SensorConfig";

        public const UInt32 SerialPortBaudRate = 115200;
        public const UInt32 SerialPortWriteTimeout = 250;
        public const UInt32 SerialPortReadTimeout = 250;
        public const int SerialPortSleepTime = 100;
        public const int SerialPortShutDownLoopDelay = 100;
        public const UInt32 SerialPortStreamBlockSize = 16;
        public const int SerialPortStreamSleepMin = 1;
        public const int SerialPortStreamSleepMax = 1;
        public const int SerialPortErrorCountMax = 10;

        public const byte StartDataFrameByte = 0x02;
        public const byte StopDataFrameByte = 0x03;
        public const byte StartInfoFrameByte = 0x3F;
        public const byte FrameEscapeByte = 0x10;

        public const UInt32 ProcessingBufferSize = 1048576; // 1 MiB
        public const UInt32 DataBlockSize = 8; // Size of the data block in bytes. 8 bytes = 2 x 32 bit
        public const UInt32 InfoBlockSize = 4;

        public const int StateHandlerSleepTime = 10; // in ms
        public const int StateHandlerCellHeatInitialTime = 5000; // 5 seconds
        public const int StateHandlerLaserHeatSweepTime = 1000; // 1 second
        public const int StateHandlerADCColdDelay = 1000; // 1 second

        public const double SensorTargetLaserTransmissionSweep = 0.2;
        public const double SensorTargetLaserTransmissionStep = 0.5;

        public const double SensorADCRawToVolts = (double)5 / 125 / 16777215;
        public const double SensorADCColdValueLowGainMinVolts = 0.5; // Minimum ADC voltage on low gain

        public const double SensorCoilsCalibrationTeslaPerHex = 21e-12; // 21 pT per step

        public const int MaxNumberOfLaserLockSweepCycles = 30;
        public const int MaxNumberOfLaserLockStepCycles = 3;
        public const int NumberOfFieldZeroingSteps = 100;
        public const int NumberOfMagnetometerCalibrationSteps = 100;

        public const int SensorLaserHeatStepCycleDelay = 2000;
        public const int SensorLaserHeatStepSleepTime = 100;

        public const string SensorCommandLaserLock = "@0";
        public const ushort SensorLaserLockDisable = 0x0000;
        public const ushort SensorLaserLockEnable = 0x0005;
        public const ushort SensorDefaultCellHeatLockPoint = 0x8000;

        public const string SensorCommandLaserCurrent = "@3";
        public const ushort SensorColdLaserCurrent = 0x0000;
        public const ushort SensorDefaultLaserCurrent = 0x9000;

        public const string SensorCommandLaserCurrentMod = "@4";
        public const ushort SensorLaserCurrentModValue = 0x0250;

        public const string SensorCommandLaserHeat = "@5";
        public const ushort SensorColdLaserHeat = 0x0000;
        public const ushort SensorDefaultLaserHeat = 0x0500;
        public const ushort SensorMinLaserHeat = 0x0000;
        public const ushort SensorMaxLaserHeat = 0x2000;
        public const ushort SensorLaserHeatStep = 10;

        public const string SensorCommandFieldXOffset = "@7";
        public const ushort SensorColdFieldXOffset = 0x8000;

        public const string SensorCommandFieldXModulationAmplitude = "@8";
        public const ushort SensorColdFieldXModulationAmplitude = 0x0000;

        public const string SensorCommandFieldYOffset = "@9";
        public const ushort SensorColdFieldYOffset = 0x8000;

        public const string SensorCommandFieldYModulationAmplitude = "@A";
        public const ushort SensorColdFieldYModulationAmplitude = 0x0000;

        public const string SensorCommandFieldZOffset = "@B";
        public const ushort SensorColdFieldZOffset = 0x8000;
        public const ushort SensorLaserLockFieldZOffset = 0x0000;

        public const string SensorCommandFieldZModulationAmplitude = "@C";
        public const ushort SensorColdFieldZModulationAmplitude = 0x0000;
        public const ushort SensorDefaultFieldZModulationAmplitude = 0x07C0;

        public const ushort SensorFieldCheckRange = 0x00EE; // about 5 nT assuming 21 pT per hex step
        public const ushort SensorFieldStep = 0x0018; // about 0.5 nT assuming 21 pT per hex step

        public const string SensorCommandDigitalDataStreamingAndGain = "@20";
        public const ushort SensorDigitalDataStreamingOffGainLow = 0x0000;
        public const ushort SensorDigitalDataStreamingOffGainHigh = 0x0001;
        public const ushort SensorDigitalDataStreamingOnGainLow = 0x0002;
        public const ushort SensorDigitalDataStreamingOnGainHigh = 0x0003;

        public const string SensorCommandCellHeat = "@21";
        public const ushort SensorColdCellHeat = 0x0000;
        public const ushort SensorDefaultCellHeat = 0x5000;
        public const ushort SensorMinCellHeat = 0x0000;
        public const ushort SensorMaxCellHeat = 0x9000;
        public const ushort SensorCellHeatStep = 10;

        public const string SensorCommandDigitalDataSelector = "@22";
        public const ushort SensorDigitalDataSelectorADC = 0x0000;
        public const ushort SensorDigitalDataSelectorZDemod = 0x0005;

        public enum SensorState : byte
        {
            Init,
            Valid,
            MakeCold,
            Cold,
            LockStart,
            LaserLockSweep,
            LaserLockStep,
            LaserLockPID,
            StabilizeCellHeat,
            ZeroFields,
            CalibrateMagnetometer,
            Idle,
            Start,
            Run,
            Stop,
            Failed,
            ShutDown
        };

    }
}
