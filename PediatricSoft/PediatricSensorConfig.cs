using System.Reflection;

namespace PediatricSoft
{
    public class PediatricSensorConfig
    {
        public ushort Chassis { get; set; }
        public ushort Port { get; set; }
        public ushort Head { get; set; }

        public ushort LaserCurrent { get; set; }
        public ushort FieldZModulationAmplitude { get; set; }
        public ushort DefaultCellHeat { get; set; }
        public ushort MaxCellHeat { get; set; }
        public ushort CellHeatLockPoint { get; set; }

        public PediatricSensorConfig()
        {
            Chassis = 0;
            Port = 0;
            Head = 0;

            LaserCurrent = PediatricSoftConstants.SensorDefaultLaserCurrent;
            FieldZModulationAmplitude = PediatricSoftConstants.SensorDefaultFieldZModulationAmplitude;
            DefaultCellHeat = PediatricSoftConstants.SensorDefaultCellHeat;
            MaxCellHeat = PediatricSoftConstants.SensorMaxCellHeat;
            CellHeatLockPoint = PediatricSoftConstants.SensorDefaultCellHeatLockPoint;
        }

        public bool Equals(PediatricSensorConfig config)
        {
            bool result = true;
            foreach (PropertyInfo pi in this.GetType().GetProperties())
            {
                //result = result && Convert.ChangeType(pi.GetValue(this), pi.PropertyType).Equals(Convert.ChangeType(pi.GetValue(config), pi.PropertyType));
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
