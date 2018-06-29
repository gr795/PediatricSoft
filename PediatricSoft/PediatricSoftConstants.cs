namespace PediatricSoft
{
    public static class PediatricSoftConstants
    {
        public static readonly int DefaultSerialPortBaudRate = 115200;
        public static readonly int DefaultSerialPortWriteTimeout = 100;
        public static readonly int DefaultSerialPortReadTimeout = 100;
        public static readonly int DefaultSerialPortSleepTime = 100;
        public static readonly int DefaultSerialPortShutDownLoopDelay = 1;
        public static readonly string ValidIDN = "PediatricSensor";

        public static readonly int MaxQueueLength = 1000;
        public static bool IsDebugEnabled = true;
        public static bool IsPlotting = false;
        public static bool PlotWindowClosed = false;
    }
}