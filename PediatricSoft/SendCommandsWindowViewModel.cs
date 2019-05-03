using Prism.Commands;
using Prism.Mvvm;
using System;
using System.Windows.Input;

namespace PediatricSoft
{
    public class SendCommandsWindowViewModel : BindableBase
    {
        public PediatricSensorData PediatricSensorData { get { return PediatricSensorData.Instance; } }
        public DebugLog DebugLog { get { return DebugLog.Instance; } }

        public PediatricSensor CurrentSensor { get; set; }

        public DelegateCommand<object> TextBoxCommandStringKeyDownCommand { get; private set; }
        public DelegateCommand<object> TextBoxCommandStringKeyUpCommand { get; private set; }
        public DelegateCommand ComboBoxCommandSelectionChangedCommand { get; private set; }
        public DelegateCommand ButtonSensorStandbyCommand { get; private set; }
        public DelegateCommand ButtonSendVCSELBurnInCommandsCommand { get; private set; }
        public DelegateCommand ButtonSwitchMagnetometerModeCommand { get; private set; }

        public TextBoxSensorConfig TextBoxChassis { get; private set; }
        public TextBoxSensorConfig TextBoxPort { get; private set; }
        public TextBoxSensorConfig TextBoxHead { get; private set; }

        public TextBoxSensorConfig TextBoxLaserCurrent { get; private set; }
        public TextBoxSensorConfig TextBoxLaserCurrentModulation { get; private set; }
        public TextBoxSensorConfig TextBoxLaserCurrentKI { get; private set; }
        public TextBoxSensorConfig TextBoxLaserHeatKI { get; private set; }
        public TextBoxSensorConfig TextBoxBzModulation { get; private set; }
        public TextBoxSensorConfig TextBoxBzKI { get; private set; }
        public TextBoxSensorConfig TextBoxDefaultCellHeat { get; private set; }
        public TextBoxSensorConfig TextBoxMaxCellHeat { get; private set; }
        public TextBoxSensorConfig TextBoxCellHeatKI { get; private set; }

        public bool RadioButtonDataSelectADCIsChecked
        {
            get
            {
                return PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.ADC;
            }
            set
            {
                PediatricSensorData.DataSelect = PediatricSoftConstants.DataSelect.ADC;
                RadioButtonsRaisePropertyChanged();
            }
        }

        public bool RadioButtonDataSelectOpenLoopIsChecked
        {
            get
            {
                return PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.OpenLoop;
            }
            set
            {
                PediatricSensorData.DataSelect = PediatricSoftConstants.DataSelect.OpenLoop;
                RadioButtonsRaisePropertyChanged();
            }
        }

        public bool RadioButtonDataSelectClosedLoopIsChecked
        {
            get
            {
                return PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.ClosedLoop;
            }
            set
            {
                PediatricSensorData.DataSelect = PediatricSoftConstants.DataSelect.ClosedLoop;
                RadioButtonsRaisePropertyChanged();
            }
        }

        public bool RadioButtonDataSelectDoubleFIsChecked
        {
            get
            {
                return PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.DoubleF;
            }
            set
            {
                PediatricSensorData.DataSelect = PediatricSoftConstants.DataSelect.DoubleF;
                RadioButtonsRaisePropertyChanged();
            }
        }

        private string textBoxCommandStringText = String.Empty;
        public string TextBoxCommandStringText
        {
            get { return textBoxCommandStringText; }
            set { textBoxCommandStringText = value; RaisePropertyChanged(); }
        }

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
                {
                    return new string[] { string.Empty };
                }
            }
        }

        public SendCommandsWindowViewModel()
        {

            TextBoxCommandStringKeyDownCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyDown);
            TextBoxCommandStringKeyUpCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyUp);
            ComboBoxCommandSelectionChangedCommand = new DelegateCommand(ComboBoxCommandOnSelectionChanged);
            ButtonSensorStandbyCommand = new DelegateCommand(ButtonSensorStandbyOnClick);
            ButtonSendVCSELBurnInCommandsCommand = new DelegateCommand(ButtonSendVCSELBurnInCommandsOnClick);
            ButtonSwitchMagnetometerModeCommand = new DelegateCommand(ButtonSwitchMagnetometerModeOnClick);
        }

        private void ComboBoxCommandOnSelectionChanged()
        {
            RaisePropertyChanged("CommandHistory");

            if (CurrentSensor != null)
            {
                TextBoxChassis = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "Chassis", false);
                RaisePropertyChanged("TextBoxChassis");

                TextBoxPort = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "Port", false);
                RaisePropertyChanged("TextBoxPort");

                TextBoxHead = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "Head", false);
                RaisePropertyChanged("TextBoxHead");

                TextBoxLaserCurrent = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "LaserCurrent");
                RaisePropertyChanged("TextBoxLaserCurrent");

                TextBoxLaserCurrentModulation = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "LaserCurrentModulation");
                RaisePropertyChanged("TextBoxLaserCurrentModulation");

                TextBoxLaserCurrentKI = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "LaserCurrentKI");
                RaisePropertyChanged("TextBoxLaserCurrentKI");

                TextBoxLaserHeatKI = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "LaserHeatKI");
                RaisePropertyChanged("TextBoxLaserHeatKI");

                TextBoxBzModulation = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "BzModulation");
                RaisePropertyChanged("TextBoxBzModulation");

                TextBoxBzKI = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "BzKI");
                RaisePropertyChanged("TextBoxBzKI");

                TextBoxDefaultCellHeat = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "DefaultCellHeat");
                RaisePropertyChanged("TextBoxDefaultCellHeat");

                TextBoxMaxCellHeat = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "MaxCellHeat");
                RaisePropertyChanged("TextBoxMaxCellHeat");

                TextBoxCellHeatKI = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "CellHeatKI");
                RaisePropertyChanged("TextBoxCellHeatKI");

            }
        }

        private void TextBoxCommandStringOnKeyDown(object parameter)
        {
            KeyEventArgs e = (KeyEventArgs)parameter;

            if (CurrentSensor != null && PediatricSensorData.CanSendCommands)
            {
                switch (e.Key)
                {
                    case Key.Add:
                        CurrentSensor.SendCommand("+");
                        TextBoxCommandStringText = String.Empty;
                        break;

                    case Key.Subtract:
                        CurrentSensor.SendCommand("-");
                        TextBoxCommandStringText = String.Empty;
                        break;

                    case Key.OemPlus:
                        CurrentSensor.SendCommand("+");
                        TextBoxCommandStringText = String.Empty;
                        break;

                    case Key.OemMinus:
                        CurrentSensor.SendCommand("-");
                        TextBoxCommandStringText = String.Empty;
                        break;

                    case Key.Enter:
                        CurrentSensor.SendCommand(TextBoxCommandStringText);
                        TextBoxCommandStringText = String.Empty;
                        RaisePropertyChanged("CommandHistory");
                        break;

                    default:
                        break;
                }
            }
            else
            {
                if (e.Key == Key.Enter && CurrentSensor == null)
                {
                    DebugLog.Enqueue($"Can't send commands - sensor sensor not selected");
                    TextBoxCommandStringText = String.Empty;
                }
                if (e.Key == Key.Enter && !PediatricSensorData.CanSendCommands)
                {
                    DebugLog.Enqueue($"Can't send commands - other operations are running");
                    TextBoxCommandStringText = String.Empty;
                }
            }
        }

        private void TextBoxCommandStringOnKeyUp(object parameter)
        {
            KeyEventArgs e = (KeyEventArgs)parameter;

            switch (e.Key)
            {
                case Key.Add:
                    TextBoxCommandStringText = String.Empty;
                    break;

                case Key.Subtract:
                    TextBoxCommandStringText = String.Empty;
                    break;

                case Key.OemPlus:
                    TextBoxCommandStringText = String.Empty;
                    break;

                case Key.OemMinus:
                    TextBoxCommandStringText = String.Empty;
                    break;

                default:
                    RaisePropertyChanged("CommandHistory");
                    break;
            }
        }

        private void ButtonSensorStandbyOnClick()
        {
            if (CurrentSensor != null)
            {
                CurrentSensor.Standby();
            }
            else
            {
                DebugLog.Enqueue("Error: sensor not selected");
            }
        }

        private void ButtonSendVCSELBurnInCommandsOnClick()
        {
            foreach (PediatricSensor sensor in PediatricSensorData.Sensors)
            {
                sensor.SendCommand(PediatricSoftConstants.SensorCommandLaserCurrent);
                sensor.SendCommand(String.Concat("#", PediatricSensor.UInt16ToStringBE(ushort.MaxValue)));
            }
        }

        private void ButtonSwitchMagnetometerModeOnClick()
        {
            if (PediatricSensorData.MagnetometerMode == PediatricSoftConstants.MagnetometerMode.OpenLoop)
            {
                PediatricSensorData.MagnetometerMode = PediatricSoftConstants.MagnetometerMode.ClosedLoop;
            }
            else
            {
                PediatricSensorData.MagnetometerMode = PediatricSoftConstants.MagnetometerMode.OpenLoop;
            }

            foreach (PediatricSensor sensor in PediatricSensorData.Sensors)
            {
                sensor.SwitchMagnetometerMode();
            }
        }

        private void RadioButtonsRaisePropertyChanged()
        {
            RaisePropertyChanged("RadioButtonDataSelectADCIsChecked");
            RaisePropertyChanged("RadioButtonDataSelectOpenLoopIsChecked");
            RaisePropertyChanged("RadioButtonDataSelectClosedLoopIsChecked");
            RaisePropertyChanged("RadioButtonDataSelectDoubleFIsChecked");
        }

    }
}
