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
        private readonly bool hexString;

        // Constructors
        public TextBoxSensorConfig(PediatricSensorConfig _config, string propertyName, bool _hexString = true)
        {
            config = _config;
            color = new SolidColorBrush(Colors.Black);
            hexString = _hexString;

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
                    if (hexString)
                    {
                        return PediatricSensor.UInt16ToStringBE((ushort)propertyInfo.GetValue(config));
                    }
                    else
                    {
                        return propertyInfo.GetValue(config).ToString();
                    }
                }
                else
                {
                    return string.Empty;
                }
            }
            set
            {
                ushort result = 0;
                bool success = false;

                try
                {
                    if (hexString)
                    {
                        result = ushort.Parse(value, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture);
                    }
                    else
                    {
                        result = ushort.Parse(value);
                    }
                    success = true;
                }
                catch { }

                if (success)
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
