namespace PediatricSoft
{
    public struct SensorScanItem
    {
        public bool isValid;
        public string port;
        public string idn;
        public string sn;
    }

    struct DataPoint
    {
        public double x;
        public double y;

        public DataPoint(double _x, double _y)
        {
            x = _x;
            y = _y;
        }
    }

}