namespace PediatricSoft
{
    public struct DataPoint
    {
        public int TimeRAW;
        public int ADCRAW;
        public int BzDemodRAW;
        public int BzErrorRAW;

        public double Time;
        public double ADC;
        public double BzDemod;
        public double BzError;

        public DataPoint(int _TimeRAW, int _ADCRAW, int _BzDemodRAW, int _BzErrorRAW,
                         double _Time, double _ADC, double _BzDemod, double _BzError)
        {
            TimeRAW = _TimeRAW;
            ADCRAW = _ADCRAW;
            BzDemodRAW = _BzDemodRAW;
            BzErrorRAW = _BzErrorRAW;

            Time = _Time;
            ADC = _ADC;
            BzDemod = _BzDemod;
            BzError = _BzError;
        }
    }
}