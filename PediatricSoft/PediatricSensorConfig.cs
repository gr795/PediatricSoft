using Prism.Mvvm;
using System.Reflection;

namespace PediatricSoft
{
    // This class contains configuration parameters that are individual to each sensor
    // Name, Chassis, Port and Head properties raise events on change. This is to auto update the main window table.

    public class PediatricSensorConfig : BindableBase
    {

        private string name;
        public string Name
        {
            get { return name; }
            set { name = value; RaisePropertyChanged(); }
        }

        private ushort chassis;
        public ushort Chassis
        {
            get { return chassis; }
            set { chassis = value; RaisePropertyChanged(); }
        }

        private ushort port;
        public ushort Port
        {
            get { return port; }
            set { port = value; RaisePropertyChanged(); }
        }

        private ushort head;
        public ushort Head
        {
            get { return head; }
            set { head = value; RaisePropertyChanged(); }
        }

        public ushort LaserCurrent { get; set; }
        public ushort LaserCurrentModulation { get; set; }
        public ushort LaserCurrentKI { get; set; }
        public ushort LaserHeatKI { get; set; }
        public ushort BzModulation { get; set; }
        public ushort BzKI { get; set; }
        public ushort DefaultCellHeat { get; set; }
        public ushort MaxCellHeat { get; set; }
        public ushort CellHeatKI { get; set; }

        // We assign default parameters on creation.
        public PediatricSensorConfig()
        {
            Name = string.Empty;

            Chassis = 0;
            Port = 0;
            Head = 0;

            LaserCurrent = PediatricSoftConstants.SensorDefaultLaserCurrent;
            LaserCurrentModulation = PediatricSoftConstants.SensorDefaultLaserCurrentModulation;
            LaserCurrentKI = PediatricSoftConstants.SensorDefaultPIDLaserCurrentI;
            LaserHeatKI = PediatricSoftConstants.SensorDefaultPIDLaserHeaterI;
            BzModulation = PediatricSoftConstants.SensorDefaultBzModulation;
            BzKI = PediatricSoftConstants.SensorDefaultPIDBzI;
            DefaultCellHeat = PediatricSoftConstants.SensorDefaultCellHeat;
            MaxCellHeat = PediatricSoftConstants.SensorMaxCellHeat;
            CellHeatKI = PediatricSoftConstants.SensorDefaultPIDCellHeaterI;
        }

        // We overload Equals method to get a value-based comparison
        // Use Reflection to iterate over all public properties
        public bool Equals(PediatricSensorConfig config)
        {
            bool result = true;
            foreach (PropertyInfo pi in this.GetType().GetProperties())
            {
                result = result && pi.GetValue(this).Equals(pi.GetValue(config));
            }
            return result;
        }

        // This method returns a new instance of PediatricSensorConfig class with the same values of all public properties
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
