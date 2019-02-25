using Prism.Commands;
using Prism.Mvvm;
using System;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Windows.Input;
using System.Windows.Media;

namespace PediatricSoft
{
    public class SendCommandsWindowViewModel : BindableBase
    {
        private PediatricSensorData PediatricSensorData;

        public ObservableCollection<PediatricSensor> Sensors { get { return PediatricSensorData.Sensors; } }
        public PediatricSensor CurrentSensor { get; set; }

        public DelegateCommand<object> TextBoxCommandStringKeyDownCommand { get; private set; }
        public DelegateCommand<object> TextBoxCommandStringKeyUpCommand { get; private set; }
        public DelegateCommand ComboBoxCommandSelectionChangedCommand { get; private set; }

        public string TextBoxCommandStringText { get; set; } = String.Empty;
        public string[] CommandHistory
        {
            get
            {
                if (CurrentSensor != null)
                {
                    string[] result = CurrentSensor.CommandHistory.ToArray();
                    Array.Reverse(result);
                    return result;
                }   
                else
                    return new string[] { string.Empty };
            }
        }

        private Brush textBoxLaserCurrentColor = new SolidColorBrush(Colors.Black);
        public Brush TextBoxLaserCurrentColor
        {
            get { return textBoxLaserCurrentColor; }
            set { textBoxLaserCurrentColor = value; RaisePropertyChanged(); }
        }

        public string TextBoxLaserCurrentText
        {
            get
            {
                if (CurrentSensor != null)
                    return PediatricSensor.UInt16ToStringBE(CurrentSensor.PediatricSensorConfig.LaserCurrent);
                else
                    return string.Empty;
            }
            set
            {
                if (ushort.TryParse(value, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort result))
                {
                    if (CurrentSensor != null)
                    {
                        CurrentSensor.PediatricSensorConfig.LaserCurrent = result;
                        TextBoxLaserCurrentColor = new SolidColorBrush(Colors.DarkGreen);
                    }
                }
                else
                {
                    TextBoxLaserCurrentColor = new SolidColorBrush(Colors.Red);
                }
                RaisePropertyChanged();
            }
        }

        private Brush textBoxFieldZModulationAmplitudeColor = new SolidColorBrush(Colors.Black);
        public Brush TextBoxFieldZModulationAmplitudeColor
        {
            get { return textBoxFieldZModulationAmplitudeColor; }
            set { textBoxFieldZModulationAmplitudeColor = value; RaisePropertyChanged(); }
        }

        public string TextBoxFieldZModulationAmplitudeText
        {
            get
            {
                if (CurrentSensor != null)
                    return PediatricSensor.UInt16ToStringBE(CurrentSensor.PediatricSensorConfig.FieldZModulationAmplitude);
                else
                    return string.Empty;
            }
            set
            {
                if (ushort.TryParse(value, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort result))
                {
                    if (CurrentSensor != null)
                    {
                        CurrentSensor.PediatricSensorConfig.FieldZModulationAmplitude = result;
                        TextBoxFieldZModulationAmplitudeColor = new SolidColorBrush(Colors.DarkGreen);
                    }
                }
                else
                {
                    TextBoxFieldZModulationAmplitudeColor = new SolidColorBrush(Colors.Red);
                }
                RaisePropertyChanged();
            }
        }

        private Brush textBoxDefaultCellHeatColor = new SolidColorBrush(Colors.Black);
        public Brush TextBoxDefaultCellHeatColor
        {
            get { return textBoxDefaultCellHeatColor; }
            set { textBoxDefaultCellHeatColor = value; RaisePropertyChanged(); }
        }

        public string TextBoxDefaultCellHeatText
        {
            get
            {
                if (CurrentSensor != null)
                    return PediatricSensor.UInt16ToStringBE(CurrentSensor.PediatricSensorConfig.DefaultCellHeat);
                else
                    return string.Empty;
            }
            set
            {
                if (ushort.TryParse(value, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort result))
                {
                    if (CurrentSensor != null)
                    {
                        CurrentSensor.PediatricSensorConfig.DefaultCellHeat = result;
                        TextBoxDefaultCellHeatColor = new SolidColorBrush(Colors.DarkGreen);
                    }
                }
                else
                {
                    TextBoxDefaultCellHeatColor = new SolidColorBrush(Colors.Red);
                }
                RaisePropertyChanged();
            }
        }

        private Brush textBoxMaxCellHeatColor = new SolidColorBrush(Colors.Black);
        public Brush TextBoxMaxCellHeatColor
        {
            get { return textBoxMaxCellHeatColor; }
            set { textBoxMaxCellHeatColor = value; RaisePropertyChanged(); }
        }

        public string TextBoxMaxCellHeatText
        {
            get
            {
                if (CurrentSensor != null)
                    return PediatricSensor.UInt16ToStringBE(CurrentSensor.PediatricSensorConfig.MaxCellHeat);
                else
                    return string.Empty;
            }
            set
            {
                if (ushort.TryParse(value, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort result))
                {
                    if (CurrentSensor != null)
                    {
                        CurrentSensor.PediatricSensorConfig.MaxCellHeat = result;
                        TextBoxMaxCellHeatColor = new SolidColorBrush(Colors.DarkGreen);
                    }
                }
                else
                {
                    TextBoxMaxCellHeatColor = new SolidColorBrush(Colors.Red);
                }
                RaisePropertyChanged();
            }
        }

        private Brush textBoxCellHeatLockPointColor = new SolidColorBrush(Colors.Black);
        public Brush TextBoxCellHeatLockPointColor
        {
            get { return textBoxCellHeatLockPointColor; }
            set { textBoxCellHeatLockPointColor = value; RaisePropertyChanged(); }
        }

        public string TextBoxCellHeatLockPointText
        {
            get
            {
                if (CurrentSensor != null)
                    return PediatricSensor.UInt16ToStringBE(CurrentSensor.PediatricSensorConfig.CellHeatLockPoint);
                else
                    return string.Empty;
            }
            set
            {
                if (ushort.TryParse(value, System.Globalization.NumberStyles.HexNumber, System.Globalization.CultureInfo.InvariantCulture, out ushort result))
                {
                    if (CurrentSensor != null)
                    {
                        CurrentSensor.PediatricSensorConfig.CellHeatLockPoint = result;
                        TextBoxCellHeatLockPointColor = new SolidColorBrush(Colors.DarkGreen);
                    }
                }
                else
                {
                    TextBoxCellHeatLockPointColor = new SolidColorBrush(Colors.Red);
                }
                RaisePropertyChanged();
            }
        }

        public SendCommandsWindowViewModel()
        {
            PediatricSensorData = PediatricSensorData.Instance;

            TextBoxCommandStringKeyDownCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyDown);
            TextBoxCommandStringKeyUpCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyUp);
            ComboBoxCommandSelectionChangedCommand = new DelegateCommand(ComboBoxCommandOnSelectionChanged);
        }

        private void ComboBoxCommandOnSelectionChanged()
        {
            RaisePropertyChanged("CommandHistory");

            TextBoxLaserCurrentColor = new SolidColorBrush(Colors.Black);
            RaisePropertyChanged("TextBoxLaserCurrentText");
            RaisePropertyChanged("TextBoxLaserCurrentColor");

            TextBoxFieldZModulationAmplitudeColor = new SolidColorBrush(Colors.Black);
            RaisePropertyChanged("TextBoxFieldZModulationAmplitudeText");
            RaisePropertyChanged("TextBoxFieldZModulationAmplitudeColor");

            TextBoxDefaultCellHeatColor = new SolidColorBrush(Colors.Black);
            RaisePropertyChanged("TextBoxDefaultCellHeatText");
            RaisePropertyChanged("TextBoxDefaultCellHeatColor");

            TextBoxMaxCellHeatColor = new SolidColorBrush(Colors.Black);
            RaisePropertyChanged("TextBoxMaxCellHeatText");
            RaisePropertyChanged("TextBoxMaxCellHeatColor");

            TextBoxCellHeatLockPointColor = new SolidColorBrush(Colors.Black);
            RaisePropertyChanged("TextBoxCellHeatLockPointText");
            RaisePropertyChanged("TextBoxCellHeatLockPointColor");
        }

        private void TextBoxCommandStringOnKeyDown(object parameter)
        {
            KeyEventArgs e = (KeyEventArgs)parameter;

            switch (e.Key)
            {
                case Key.Add:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("+");
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                case Key.Subtract:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("-");
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                case Key.OemPlus:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("+");
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                case Key.OemMinus:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("-");
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                case Key.Enter:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand(TextBoxCommandStringText);
                    else
                        Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Can't send commands - sensor sensor not selected");
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    RaisePropertyChanged("CommandHistory");
                    break;

                default:
                    break;
            }
        }

        private void TextBoxCommandStringOnKeyUp(object parameter)
        {
            KeyEventArgs e = (KeyEventArgs)parameter;

            switch (e.Key)
            {
                case Key.Add:
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                case Key.Subtract:
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                case Key.OemPlus:
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                case Key.OemMinus:
                    TextBoxCommandStringText = String.Empty;
                    RaisePropertyChanged("TextBoxCommandStringText");
                    break;

                default:
                    RaisePropertyChanged("CommandHistory");
                    break;
            }
        }

    }
}
