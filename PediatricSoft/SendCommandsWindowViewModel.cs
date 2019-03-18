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

        public ObservableCollection<PediatricSensor> Sensors { get { return PediatricSensorData.Sensors; } }
        public PediatricSensor CurrentSensor { get; set; }

        public DelegateCommand<object> TextBoxCommandStringKeyDownCommand { get; private set; }
        public DelegateCommand<object> TextBoxCommandStringKeyUpCommand { get; private set; }
        public DelegateCommand ComboBoxCommandSelectionChangedCommand { get; private set; }
        public DelegateCommand ButtonSendSetupCommandsCommand { get; private set; }

        public TextBoxSensorConfig TextBoxLaserCurrent { get; private set; }
        public TextBoxSensorConfig TextBoxFieldZModulationAmplitude { get; private set; }
        public TextBoxSensorConfig TextBoxDefaultCellHeat { get; private set; }
        public TextBoxSensorConfig TextBoxMaxCellHeat { get; private set; }
        public TextBoxSensorConfig TextBoxCellHeatLockPoint { get; private set; }

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
                TextBoxLaserCurrent = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "LaserCurrent");
                RaisePropertyChanged("TextBoxLaserCurrent");

                TextBoxFieldZModulationAmplitude = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "FieldZModulationAmplitude");
                RaisePropertyChanged("TextBoxFieldZModulationAmplitude");

                TextBoxDefaultCellHeat = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "DefaultCellHeat");
                RaisePropertyChanged("TextBoxDefaultCellHeat");

                TextBoxMaxCellHeat = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "MaxCellHeat");
                RaisePropertyChanged("TextBoxMaxCellHeat");

                TextBoxCellHeatLockPoint = new TextBoxSensorConfig(CurrentSensor.PediatricSensorConfig, "CellHeatLockPoint");
                RaisePropertyChanged("TextBoxCellHeatLockPoint");
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
                        if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Can't send commands - sensor sensor not selected");
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
            if (CurrentSensor != null) CurrentSensor.SendCommandsSetup();
        }

    }
}
