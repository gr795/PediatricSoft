using System;

namespace PediatricSoft
{
    public class XYPoint
    {

        private double x;
        private double y;
        private double sumY;
        private int numY;

        public const double Base = 10;

        public double X
        {
            get { return x; }
            set { x = value; }
        }

        public double Y
        {
            get { return y; }
            set
            {
                y = value;
                sumY += value;
                numY++;
            }
        }

        public double LogY
        {
            get
            {
                double t = Math.Log(Y, Base);
                if (!double.IsInfinity(t))
                {
                    return t;
                }
                else
                {
                    return 0;
                }
            }
        }

        public double LogYAvg
        {
            get
            {
                double t = Math.Log(sumY / numY, Base);
                if (!double.IsInfinity(t))
                {
                    return t;
                }
                else
                {
                    return 0;
                }
            }
        }

        public XYPoint(double _x, double _y)
        {
            X = _x;
            Y = _y;
        }


    }
}
