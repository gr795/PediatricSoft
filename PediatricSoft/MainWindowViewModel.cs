using Prism.Commands;
using Prism.Mvvm;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace PediatricSoft
{
    public class MainWindowViewModel : BindableBase
    {

        //PlotWindow plotWindow = new PlotWindow();
        //SendCommandsWindow sendCommandsWindow = new SendCommandsWindow();

        private PediatricSensorData PediatricSensorData;

        public DelegateCommand ButtonScanPortsCommand { get; private set; }
        public DelegateCommand ButtonLockSensorsCommand { get; private set; }
        public DelegateCommand ButtonStartStopSensorsCommand { get; private set; }
        public DelegateCommand ButtonZeroFieldsCommand { get; private set; }




        private string buttonPlot_Content = "Show Plot";
        public string ButtonPlot_Content
        {
            get { return buttonPlot_Content; }
            private set { buttonPlot_Content = value; RaisePropertyChanged(); }
        }

        private bool saveRAWValuesCheckbox_IsEnabled = false;
        public bool SaveRAWValuesCheckbox_IsEnabled
        {
            get { return saveRAWValuesCheckbox_IsEnabled; }
            private set { saveRAWValuesCheckbox_IsEnabled = value; RaisePropertyChanged(); }
        }

        public bool SaveRAWValuesCheckbox_IsChecked
        {
            get { return PediatricSensorData.SaveRAWValues; }
            set { PediatricSensorData.SaveRAWValues = value; }
        }

        private bool saveDataCheckbox_IsEnabled = false;
        public bool SaveDataCheckbox_IsEnabled
        {
            get { return saveDataCheckbox_IsEnabled; }
            private set { saveDataCheckbox_IsEnabled = value; RaisePropertyChanged(); }
        }

        public bool SaveDataCheckbox_IsChecked
        {
            get { return PediatricSensorData.SaveDataEnabled; }
            set { PediatricSensorData.SaveDataEnabled = value; }
        }

        private bool saveSuffixTextBox_IsEnabled = false;
        public bool SaveSuffixTextBox_IsEnabled
        {
            get { return saveSuffixTextBox_IsEnabled; }
            private set { saveSuffixTextBox_IsEnabled = value; RaisePropertyChanged(); }
        }

        public string SaveSuffixTextBox_Text
        {
            get { return PediatricSensorData.SaveSuffix; }
            set { PediatricSensorData.SaveSuffix = value; }
        }





        private string buttonStartStopSensorsContent = "Start Sensors";
        public string ButtonStartStopSensorsContent
        {
            get { return buttonStartStopSensorsContent; }
            private set { buttonStartStopSensorsContent = value; RaisePropertyChanged(); }
        }
        
        private void OnPediatricSensorDataPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case "SensorCount":
                    RaisePropertyChanged("SensorCount");
                    break;

                case "CanScan":
                    ButtonScanPortsCommand.RaiseCanExecuteChanged();
                    break;

                case "CanLock":
                    ButtonLockSensorsCommand.RaiseCanExecuteChanged();
                    break;

                case "CanStartStop":
                    ButtonStartStopSensorsCommand.RaiseCanExecuteChanged();
                    break;

                case "IsRunning":
                    if (PediatricSensorData.IsRunning)
                        ButtonStartStopSensorsContent = "Stop Sensors";
                    else
                        ButtonStartStopSensorsContent = "Start Sensors";
                    break;

                case "CanZeroFields":
                    ButtonZeroFieldsCommand.RaiseCanExecuteChanged();
                    break;

                default:
                    break;
            }
        }

        public MainWindowViewModel()
        {
            PediatricSensorData = PediatricSensorData.Instance;

            ButtonScanPortsCommand = new DelegateCommand(PediatricSensorData.ScanPortsAsync, PediatricSensorData.ScanPortsAsyncCanExecute);
            ButtonLockSensorsCommand = new DelegateCommand(PediatricSensorData.LockAllAsync, PediatricSensorData.LockAllAsyncCanExecute);
            ButtonStartStopSensorsCommand = new DelegateCommand(PediatricSensorData.StartStopAsync, PediatricSensorData.StartStopAsyncCanExecute);
            ButtonZeroFieldsCommand = new DelegateCommand(PediatricSensorData.ZeroFieldsAsync, PediatricSensorData.ZeroFieldsAsyncCanExecute);

            //plotWindow.Closed += OnPlotWindowClosing;
            PediatricSensorData.PropertyChanged += OnPediatricSensorDataPropertyChanged;
        }

    }
}
