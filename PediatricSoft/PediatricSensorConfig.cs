using System.Reflection;

namespace PediatricSoft
{
    public class PediatricSensorConfig
    {
        public ushort Chassis { get; set; }
        public ushort Port { get; set; }
        public ushort Head { get; set; }

        public ushort LaserCurrent { get; set; }
        public ushort LaserCurrentModulation { get; set; }
        public ushort BzModulation { get; set; }
        public ushort DefaultCellHeat { get; set; }
        public ushort MaxCellHeat { get; set; }

        public PediatricSensorConfig()
        {
            Chassis = 0;
            Port = 0;
            Head = 0;

            LaserCurrent = PediatricSoftConstants.SensorDefaultLaserCurrent;
            LaserCurrentModulation = PediatricSoftConstants.SensorDefaultLaserCurrentModulation;
            BzModulation = PediatricSoftConstants.SensorDefaultBzModulation;
            DefaultCellHeat = PediatricSoftConstants.SensorDefaultCellHeat;
            MaxCellHeat = PediatricSoftConstants.SensorMaxCellHeat;
        }

        public bool Equals(PediatricSensorConfig config)
        {
            bool result = true;
            foreach (PropertyInfo pi in this.GetType().GetProperties())
            {
                result = result && pi.GetValue(this).Equals(pi.GetValue(config));
            }
            return result;
        }

        public PediatricSensorConfig GetValueCopy()
        {
            PediatricSensorConfig config = new PediatricSensorConfig();
            foreach (PropertyInfo pi in this.GetType().GetProperties())
            {
                pi.SetValue(config, pi.GetValue(this));
            }
            return config;
        }

    }
}
