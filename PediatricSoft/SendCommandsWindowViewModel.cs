using Prism.Commands;
using Prism.Mvvm;
using System;
using System.Collections.ObjectModel;
using System.Windows.Input;

namespace PediatricSoft
{
    public class SendCommandsWindowViewModel : BindableBase
    {
        private PediatricSensorData PediatricSensorData;
        private DebugLog DebugLog = DebugLog.Instance;

        public ObservableCollection<PediatricSensor> Sensors { get { return PediatricSensorData.Sensors; } }
        public PediatricSensor CurrentSensor { get; set; }

        public DelegateCommand<object> TextBoxCommandStringKeyDownCommand { get; private set; }
        public DelegateCommand<object> TextBoxCommandStringKeyUpCommand { get; private set; }
        public DelegateCommand ComboBoxCommandSelectionChangedCommand { get; private set; }
        public DelegateCommand ButtonSendSetupCommandsCommand { get; private set; }

        public TextBoxSensorConfig TextBoxChassis { get; private set; }
        public TextBoxSensorConfig TextBoxPort { get; private set; }
        public TextBoxSensorConfig TextBoxHead { get; private set; }

        public TextBoxSensorConfig TextBoxLaserCurrent { get; private set; }
        public TextBoxSensorConfig TextBoxLaserCurrentModulation { get; private set; }
        public TextBoxSensorConfig TextBoxBzModulation { get; private set; }
        public TextBoxSensorConfig TextBoxBzKI { get; private set; }
        public TextBoxSensorConfig TextBoxDefaultCellHeat { get; private set; }
        public TextBoxSensorConfig TextBoxMaxCellHeat { get; private set; }

        public bool RadioButtonDataSelectADCIsChecked
        {
            get
            {
                if (PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.ADC)
                    return true;
                else
                    return false;
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
                if (PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.OpenLoop)
                    return true;
                else
                    return false;
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
                if (PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.ClosedLoop)
                    return true;
                else
                    return false;
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
                if (PediatricSensorData.DataSelect == PediatricSoftConstants.DataSelect.DoubleF)
                    return true;
                else
                    return false;
            }
            set
            {
                PediatricSensorData.DataSelect = PediatricSoftConstants.DataSelect.DoubleF;
                RadioButtonsRaisePropertyChanged();
            }
        }

        public bool RadioButtonDataSelectIsEnabled
        {
            get { return PediatricSensorData.DebugMode || (!PediatricSensorData.IsRunning && PediatricSensorData.CanStartStop); }
        }

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

        public SendCommandsWindowViewModel()
        {
            PediatricSensorData = PediatricSensorData.Instance;

            TextBoxCommandStringKeyDownCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyDown);
            TextBoxCommandStringKeyUpCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyUp);
            ComboBoxCommandSelectionChangedCommand = new DelegateCommand(ComboBoxCommandOnSelectionChanged);
            ButtonSendSetupCommandsCommand = new DelegateCommand(ButtonSendSetupCommandsOnClick, () => PediatricSensorData.DebugMode);
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

                TextBoxBzModulation = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "BzModulation");
                RaisePropertyChanged("TextBoxBzModulation");

                TextBoxBzKI = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "BzKI");
                RaisePropertyChanged("TextBoxBzKI");

                TextBoxDefaultCellHeat = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "DefaultCellHeat");
                RaisePropertyChanged("TextBoxDefaultCellHeat");

                TextBoxMaxCellHeat = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "MaxCellHeat");
                RaisePropertyChanged("TextBoxMaxCellHeat");
                
            }
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
                        if (PediatricSensorData.DebugMode) DebugLog.Enqueue($"Can't send commands - sensor sensor not selected");
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

        private void ButtonSendSetupCommandsOnClick()
        {
            if (CurrentSensor != null)
            {
                CurrentSensor.Standby();
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
