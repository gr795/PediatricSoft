namespace PediatricSoft
{
    public struct DataPoint
    {
        public int TimeRAW;
        public int ADCRAW;
        public int BzDemodRAW;
        public int BzFeedbackRAW;
        public int Bz2fRAW;

        public double Time;
        public double ADC;
        public double BzDemod;
        public double BzFeedback;
        public double Bz2f;

        public DataPoint(int _TimeRAW, int _ADCRAW, int _BzDemodRAW, int _BzFeedbackRAW, int _Bz2fRAW,
                         double _Time, double _ADC, double _BzDemod, double _BzFeedback, double _Bz2f)
        {
            TimeRAW = _TimeRAW;
            ADCRAW = _ADCRAW;
            BzDemodRAW = _BzDemodRAW;
            BzFeedbackRAW = _BzFeedbackRAW;
            Bz2fRAW = _Bz2fRAW;

            Time = _Time;
            ADC = _ADC;
            BzDemod = _BzDemod;
            BzFeedback = _BzFeedback;
            Bz2f = _Bz2f;
        }
    }
}