using Prism.Mvvm;
using System.Reflection;
using System.Windows.Media;

namespace PediatricSoft
{
    public class TextBoxSensorConfig : BindableBase
    {
        // Fields
        private readonly PediatricSensorConfig config;
        private readonly PropertyInfo propertyInfo;
        private Brush color;

        // Constructors
        public TextBoxSensorConfig(PediatricSensorConfig _config, string propertyName)
        {
            config = _config;
            color = new SolidColorBrush(Colors.Black);

            foreach (PropertyInfo pi in config.GetType().GetProperties())
            {
                if (string.Equals(pi.Name, propertyName))
                {
                    propertyInfo = pi;
                    break;
                }
            }
        }

        // Properties

        public Brush Color
        {
            get { return color; }
            set { color = value; RaisePropertyChanged(); }
        }

        public string Text
        {
            get
            {
                if (config != null && propertyInfo != null)
                {
                    return PediatricSensor.UInt16ToStringBE((ushort)propertyInfo.GetValue(config));
                }
                else
                {
                    return string.Empty;
                }
            }
            set
            {
                if (ushort.TryParse(value, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort result))
                {
                    if (config != null && propertyInfo != null)
                    {
                        propertyInfo.SetValue(config, result);
                        Color = new SolidColorBrush(Colors.DarkGreen);
                    }
                }
                else
                {
                    Color = new SolidColorBrush(Colors.Red);
                }
                RaisePropertyChanged();
            }
        }

        // Methods

        // Event Handlers
    }
}
