namespace PediatricSoft
{
    public struct DataPoint
    {
        public int TimeRAW;
        public int ADCRAW;
        public int BzDemodRAW;
        public int BzFeedbackRAW;
        public int OtherRAW;

        public double Time;
        public double ADC;
        public double BzDemod;
        public double BzFeedback;
        public double Other;

        public DataPoint(int _TimeRAW, int _ADCRAW, int _BzDemodRAW, int _BzFeedbackRAW, int _OtherRAW,
                         double _Time, double _ADC, double _BzDemod, double _BzFeedback, double _Other)
        {
            TimeRAW = _TimeRAW;
            ADCRAW = _ADCRAW;
            BzDemodRAW = _BzDemodRAW;
            BzFeedbackRAW = _BzFeedbackRAW;
            OtherRAW = _OtherRAW;

            Time = _Time;
            ADC = _ADC;
            BzDemod = _BzDemod;
            BzFeedback = _BzFeedback;
            Other = _Other;
        }
    }
}