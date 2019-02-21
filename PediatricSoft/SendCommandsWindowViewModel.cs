using Prism.Commands;
using Prism.Mvvm;
using System;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Windows.Input;

namespace PediatricSoft
{
    public class SendCommandsWindowViewModel : BindableBase
    {
        private PediatricSensorData PediatricSensorData;

        public DelegateCommand<object> TextBoxCommandStringKeyDownCommand { get; private set; }
        public DelegateCommand<object> TextBoxCommandStringKeyUpCommand { get; private set; }

        public string TextBoxCommandStringText { get; set; } = String.Empty;
        public string CommandHistory
        {
            get { return PediatricSensorData.CommandHistory; }
        }
        public ObservableCollection<PediatricSensor> Sensors { get { return PediatricSensorData.Sensors; } }
        public PediatricSensor CurrentSensor { get; set; }

        public SendCommandsWindowViewModel()
        {
            PediatricSensorData = PediatricSensorData.Instance;

            TextBoxCommandStringKeyDownCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyDown);
            TextBoxCommandStringKeyUpCommand = new DelegateCommand<object>(TextBoxCommandStringOnKeyUp);
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

                    PediatricSensorData.CommandHistory = String.Concat(TextBoxCommandStringText, "\n", PediatricSensorData.CommandHistory);
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
