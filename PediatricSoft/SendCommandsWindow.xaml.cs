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

        public string CommandBoxText { get; set; }
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

        private void TextBox_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                TextBox tBox = (TextBox)sender;
                DependencyProperty prop = TextBox.TextProperty;

                BindingExpression binding = BindingOperations.GetBindingExpression(tBox, prop);
                if (binding != null) { binding.UpdateSource(); }
                
                if (CurrentSensor != null)
                    CurrentSensor.SendCommand(CommandBoxText);
                else
                    Debug.WriteLineIf(PediatricSensorData.IsDebugEnabled, $"Can't send commands - sensor sensor not selected");

                CommandBoxText = String.Empty;
                OnPropertyChanged("CommandBoxText");
            }
        }

    }
}
