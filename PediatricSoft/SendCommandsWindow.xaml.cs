using System.Windows;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Collections.ObjectModel;

namespace PediatricSoft
{
    
    public partial class SendCommandsWindow : Window, INotifyPropertyChanged
    {

        PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;

        public string CommandBoxText { get; set; } = String.Empty;
        public string CommandHistory
        {
            get { return PediatricSensorData.CommandHistory; }
        }
        public ObservableCollection<PediatricSensor> Sensors { get { return PediatricSensorData.Sensors; } }
        public PediatricSensor CurrentSensor { get; set; }

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged(string prop)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop));
        }

        public SendCommandsWindow()
        {
            InitializeComponent();
            DataContext = this;
        }

        private void TextBox_KeyDown(object sender, KeyEventArgs e)
        {

            TextBox tBox = (TextBox)sender;
            DependencyProperty prop = TextBox.TextProperty;

            BindingExpression binding = BindingOperations.GetBindingExpression(tBox, prop);
            if (binding != null) { binding.UpdateSource(); }

            switch (e.Key)
            {
                case Key.Add:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("+", false);
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                case Key.Subtract:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("-", false);
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                case Key.OemPlus:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("+", false);
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                case Key.OemMinus:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand("-", false);
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                case Key.Enter:
                    if (CurrentSensor != null)
                        CurrentSensor.SendCommand(CommandBoxText);
                    else
                        Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Can't send commands - sensor sensor not selected");

                    PediatricSensorData.CommandHistory = String.Concat(CommandBoxText, "\n", PediatricSensorData.CommandHistory);
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    OnPropertyChanged("CommandHistory");
                    break;

                default:
                    break;
            }

            
        }

        private void TextBox_KeyUp(object sender, KeyEventArgs e)
        {

            TextBox tBox = (TextBox)sender;
            DependencyProperty prop = TextBox.TextProperty;

            BindingExpression binding = BindingOperations.GetBindingExpression(tBox, prop);
            if (binding != null) { binding.UpdateSource(); }

            switch (e.Key)
            {
                case Key.Add:
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                case Key.Subtract:
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                case Key.OemPlus:
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                case Key.OemMinus:
                    CommandBoxText = String.Empty;
                    OnPropertyChanged("CommandBoxText");
                    break;

                default:
                    OnPropertyChanged("CommandHistory");
                    break;
            }

            
        }
    }
}
