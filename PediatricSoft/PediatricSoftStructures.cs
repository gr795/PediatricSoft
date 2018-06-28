namespace PediatricSoft
{
    public struct SensorScanItem
    {
        public bool isValid;
        public string port;
        public string idn;
        public string sn;
    }

    public struct DataPoint
    {
        public double X;
        public double Y;

        public DataPoint(double _x, double _y)
        {
            X = _x;
            Y = _y;
        }
    }

}