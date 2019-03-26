﻿using System;

namespace PediatricSoft
{
    public static class PediatricSoftConstants
    {

        // Constants
        public const int DebugLogQueueMaxCount = 128;
        public const int DataSampleRate = 1000;
        public const int DataQueueLength = 4096; // number of data points to hold in memory and plot
        public const int PlotQueueLength = 256;
        public const int FFTLength = DataQueueLength / 2;
        public const double FFTMaxFrequency = 100;
        public const int UIUpdateInterval = 250; // Update UI every X ms
        public const string ValidIDN = "Arrow USB Blaster B";
        public const string PediatricSoftFolderRelative = "PediatricSoft";
        public const string SensorConfigFolderRelative = "SensorConfig";

        public const int SerialPortMaxRetries = 5;
        public const int SerialPortSleepAfterFail = 5000;
        public const UInt32 SerialPortBaudRate = 921600;
        public const UInt32 SerialPortWriteTimeout = 100;
        public const UInt32 SerialPortReadTimeout = 10;
        public const byte SerialPortLatency = 2;

        public const byte StartDataFrameByte = 0x02;
        public const byte StartInfoFrameByte = 0x3F;
        public const byte FrameEscapeByte = 0x10;

        public const UInt32 StreamingBufferSize = 1048576;
        public const UInt32 DataBlockSize = 20; // Size of the data block in bytes.
        public const UInt32 InfoBlockSize = 4;

        public const int StateHandlerSleepTime = 10; // in ms
        public const int StateHandlerCellHeatInitialTime = 5000; // 5 seconds
        public const int StateHandlerLaserHeatSweepTime = 1000; // 1 second
        public const int StateHandlerADCColdDelay = 1000; // 1 second
        public const int StateHandlerCellHeatStabilizeTime = 180000; // 3 minutes
        public const int StateHandlerTransmissionAveragingTime = 1000; // in ms

        public const double SensorTargetLaserTransmissionStep = 0.3;
        public const double SensorTargetLaserTransmissionRun = 0.33;
        public const double SensorTargetLaserTransmission5Percent = 0.05;

        public const double ConversionTime = (double)1 / DataSampleRate;
        public const double ConversionADC = (double)5 / 125 / 16777215;

        public const double SensorADCColdValueLowGainMinVolts = 0.5; // Minimum ADC voltage on low gain
        public const double SensorADCColdValueLowGainMaxVolts = 4.99;

        public const double SensorYCoilCalibrationTeslaPerHex = 2.29e-12; // 4.5 pT per step
        public const double SensorZCoilCalibrationTeslaPerHex = 2.29e-12; // 4.5 pT per step

        public const double StreamingAccumulatorSize = 125;
        public const int MaxNumberOfLaserLockStepCycles = 30;
        public const int NumberOfFieldZeroingIntervalsOneAxis = 20; // This produces (n+1)^2 iterations

        public const ushort SensorFieldCheckRange = 0x00DE; // about 1 nT 
        public const ushort SensorFieldStep = 0x0016; // about 0.1 nT 

        public const string SensorCommandLock = "@0";
        public const ushort SensorLockDisable = 0x0000;
        public const ushort SensorLaserLockEnable = 0x0005;
        public const ushort SensorCellLockEnable = 0x0010;
        public const ushort SensorBzLockEnable = 0x0008;
        public const ushort SensorByLockEnable = 0x0020;

        public const string SensorCommandPIDInverse = "@1";
        public const ushort SensorDefaultPIDInverse = 0x0001;

        public const string SensorCommandLaserCurrent = "@3";
        public const ushort SensorColdLaserCurrent = 0x0000;
        public const ushort SensorDefaultLaserCurrent = 0x7000;

        public const string SensorCommandLaserCurrentModulation = "@4";
        public const ushort SensorDefaultLaserCurrentModulation = 0x0100;

        public const string SensorCommandLaserHeat = "@5";
        public const ushort SensorColdLaserHeat = 0x0000;
        public const ushort SensorMinLaserHeat = 0x0000;
        public const ushort SensorMaxLaserHeat = 0x1800;
        public const ushort SensorLaserHeatStep = 0x0008;

        public const string SensorCommandFieldXOffset = "@7";
        public const ushort SensorDefaultFieldXOffset = 0x8000;

        public const string SensorCommandFieldXModulationAmplitude = "@8";
        public const ushort SensorDefaultFieldXModulationAmplitude = 0x0000;

        public const string SensorCommandFieldYOffset = "@9";
        public const ushort SensorDefaultFieldYOffset = 0x8000;

        public const string SensorCommandFieldYModulationAmplitude = "@A";
        public const ushort SensorDefaultFieldYModulationAmplitude = 0x3000;

        public const string SensorCommandFieldZOffset = "@B";
        public const ushort SensorDefaultFieldZOffset = 0x8000;

        public const string SensorCommandFieldZModulationAmplitude = "@C";
        public const ushort SensorDefaultFieldZModulationAmplitude = 0x3000;

        public const string SensorCommandLaserModulationFrequency = "@E";
        public const ushort SensorDefaultLaserModulationFrequency = 0x0010;

        public const string SensorCommandDelayForLaser = "@F";
        public const ushort SensorDefaultDelayForLaser = 0x0590;

        public const string SensorCommandPIDLaserCurrentP = "@10";
        public const ushort SensorDefaultPIDLaserCurrentP = 0x0000;

        public const string SensorCommandPIDLaserCurrentI = "@11";
        public const ushort SensorDefaultPIDLaserCurrentI = 0x0080;

        public const string SensorCommandPIDLaserHeaterI = "@1D";
        public const ushort SensorDefaultPIDLaserHeaterI = 0x0200;

        public const string SensorCommandPIDCellHeaterI = "@2A";
        public const ushort SensorDefaultPIDCellHeaterI = 0x0008;

        public const string SensorCommandPIDByP = "@16";
        public const ushort SensorDefaultPIDByP = 0x0000;

        public const string SensorCommandPIDByI = "@17";
        public const ushort SensorDefaultPIDByI = 0x0004;

        public const string SensorCommandPIDBzP = "@19";
        public const ushort SensorDefaultPIDBzP = 0x0000;

        public const string SensorCommandPIDBzI = "@1A";
        public const ushort SensorDefaultPIDBzI = 0x0004;

        public const string SensorCommandDigitalDataStreamingAndADCGain = "@20";
        public const ushort SensorDigitalDataStreamingOffGainLow = 0x0000;
        public const ushort SensorDigitalDataStreamingOffGainHigh = 0x0001;
        public const ushort SensorDigitalDataStreamingOnGainLow = 0x0002;
        public const ushort SensorDigitalDataStreamingOnGainHigh = 0x0003;

        public const string SensorCommandCellHeat = "@21";
        public const ushort SensorColdCellHeat = 0x0000;
        public const ushort SensorDefaultCellHeat = 0x8000;
        public const ushort SensorMinCellHeat = 0x0000;
        public const ushort SensorMaxCellHeat = 0xFFFF;

        public const string SensorCommandDigitalDataSelector = "@22";
        public const ushort SensorDefaultDigitalDataSelector = 0x7650;

        public const string SensorCommandADCDisplayGain = "@23";
        public const ushort SensorDefaultADCDisplayGain = 0x0009;

        public const string SensorCommandStepSize = "@24";
        public const ushort SensorDefaultStepSize = 0x0010;

        public const string SensorCommand2fPhase = "@26";
        public const ushort SensorDefault2fPhase = 0x05C0;

        public const string SensorCommandBzPhase = "@27";
        public const ushort SensorDefaultBzPhase = 0x0500;

        public const string SensorCommandCellHeatLockPoint = "@28";
        public const ushort SensorDefaultCellHeatLockPoint = 0x1000;

        public const string SensorCommandTriggers = "@40";
        public const ushort SensorSetCurrentADCValueAsHeatLockPoint = 0x0004;

        public enum SensorState : byte
        {
            Init,
            Valid,
            Setup,
            StartLock,
            LaserLockStep,
            LaserLockPID,
            CellHeatLock,
            ZeroFields,
            CalibrateMagnetometer,
            Idle,
            Start,
            Run,
            Stop,
            ShutDownRequested,
            ShutDownComplete,
            Failed
        };

        public enum DataSelect
        {
            ADC,
            OpenLoop,
            ClosedLoop
        }

    }
}
