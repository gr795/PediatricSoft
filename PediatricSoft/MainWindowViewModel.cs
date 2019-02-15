using Prism.Commands;
using Prism.Events;
using Prism.Mvvm;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Windows.Forms;

namespace PediatricSoft
{
    public class MainWindowViewModel : BindableBase
    {

        // Fields
        
        private PediatricSensorData PediatricSensorData;
        private readonly IEventAggregator eventAggregator;

        // Constructors

        public MainWindowViewModel()
        {
            PediatricSensorData = PediatricSensorData.Instance;
            eventAggregator = PediatricSoftEventGlue.eventAggregator;

            ButtonScanPortsCommand = new DelegateCommand(PediatricSensorData.ScanPortsAsync, PediatricSensorData.ScanPortsAsyncCanExecute);
            ButtonLockSensorsCommand = new DelegateCommand(PediatricSensorData.LockAllAsync, PediatricSensorData.LockAllAsyncCanExecute);
            ButtonStartStopSensorsCommand = new DelegateCommand(PediatricSensorData.StartStopAsync, PediatricSensorData.StartStopAsyncCanExecute);
            ButtonZeroFieldsCommand = new DelegateCommand(PediatricSensorData.ZeroFieldsAsync, PediatricSensorData.ZeroFieldsAsyncCanExecute);
            CheckBoxSaveDataCommand = new DelegateCommand(CheckBoxSaveDataOnToggle);
            ButtonSendCommandsCommand = new DelegateCommand(ButtonSendCommandsOnClick);

            PediatricSensorData.PropertyChanged += OnPediatricSensorDataPropertyChanged;
        }

        // Properties

        public DelegateCommand ButtonScanPortsCommand { get; private set; }
        public DelegateCommand ButtonLockSensorsCommand { get; private set; }
        public DelegateCommand ButtonStartStopSensorsCommand { get; private set; }
        public DelegateCommand ButtonZeroFieldsCommand { get; private set; }
        public DelegateCommand CheckBoxSaveDataCommand { get; private set; }
        public DelegateCommand ButtonSendCommandsCommand { get; private set; }

        public ObservableCollection<PediatricSensor> Sensors { get { return PediatricSensorData.Sensors; } }

        public bool ButtonSendCommandsIsEnabled
        {
            get { return PediatricSensorData.CanSendCommands; }
        }

        private bool checkBoxSaveDataIsEnabled = false;
        public bool CheckBoxSaveDataIsEnabled
        {
            get { return checkBoxSaveDataIsEnabled; }
            private set { checkBoxSaveDataIsEnabled = value; RaisePropertyChanged(); }
        }

        public bool CheckBoxSaveDataIsChecked
        {
            get { return PediatricSensorData.SaveDataEnabled; }
            set { PediatricSensorData.SaveDataEnabled = value; RaisePropertyChanged(); }
        }

        private bool checkBoxSaveRAWValuesIsEnabled = false;
        public bool CheckBoxSaveRAWValuesIsEnabled
        {
            get { return checkBoxSaveRAWValuesIsEnabled; }
            private set { checkBoxSaveRAWValuesIsEnabled = value; RaisePropertyChanged(); }
        }

        public bool CheckBoxSaveRAWValuesIsChecked
        {
            get { return PediatricSensorData.SaveRAWValues; }
            set { PediatricSensorData.SaveRAWValues = value; RaisePropertyChanged(); }
        }

        private bool textBlockSaveSuffixIsEnabled = false;
        public bool TextBlockSaveSuffixIsEnabled
        {
            get { return textBlockSaveSuffixIsEnabled; }
            private set { textBlockSaveSuffixIsEnabled = value; RaisePropertyChanged(); }
        }

        public string TextBlockSaveSuffixText
        {
            get { return PediatricSensorData.SaveSuffix; }
            set { PediatricSensorData.SaveSuffix = value; }
        }

        public string TextBlockSaveFolderText
        {
            get { return PediatricSensorData.SaveFolder; }
            set { PediatricSensorData.SaveFolder = value; RaisePropertyChanged(); }
        }

        public string TextBlockSensorCountText
        {
            get { return $"Sensor Count: {PediatricSensorData.SensorCount}"; }
        }

        private string buttonStartStopSensorsContent = "Start Sensors";
        public string ButtonStartStopSensorsContent
        {
            get { return buttonStartStopSensorsContent; }
            private set { buttonStartStopSensorsContent = value; RaisePropertyChanged(); }
        }

        // Methods

        private void CheckBoxSaveDataOnToggle()
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Main Window View Model: Save Data checkbox toggled");

            if (string.IsNullOrEmpty(TextBlockSaveFolderText))
                ChooseSaveDataFolder();

            if (CheckBoxSaveDataIsChecked)
            {
                CheckBoxSaveRAWValuesIsEnabled = true;
                TextBlockSaveSuffixIsEnabled = true;
            }
            else
            {
                CheckBoxSaveRAWValuesIsEnabled = false;
                TextBlockSaveSuffixIsEnabled = false;
            }
        }

        private void ChooseSaveDataFolder()
        {
            using (var dialog = new System.Windows.Forms.FolderBrowserDialog())
            {
                System.Windows.Forms.DialogResult result = dialog.ShowDialog();
                if (result == DialogResult.OK)
                {
                    TextBlockSaveFolderText = dialog.SelectedPath;
                    Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, $"Main Window View Model: Save Data Folder: {TextBlockSaveFolderText}");
                }
                else CheckBoxSaveDataIsChecked = false;
            }
        }

        private void ButtonSendCommandsOnClick()
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Main Window View Model: Send Commands Button clicked");
            eventAggregator.GetEvent<EventWindowManager>().Publish("ShowSendCommandsWindow");
        }


        // Event handlers

        private void OnPediatricSensorDataPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case "SensorCount":
                    RaisePropertyChanged("TextBlockSensorCountText");
                    break;

                case "CanScan":
                    App.Current.Dispatcher.Invoke(() => ButtonScanPortsCommand.RaiseCanExecuteChanged() );
                    break;

                case "CanLock":
                    App.Current.Dispatcher.Invoke(() => ButtonLockSensorsCommand.RaiseCanExecuteChanged());
                    break;

                case "CanStartStop":
                    App.Current.Dispatcher.Invoke(() => ButtonStartStopSensorsCommand.RaiseCanExecuteChanged());
                    if (PediatricSensorData.CanStartStop)
                        if (PediatricSensorData.IsRunning)
                        {
                            CheckBoxSaveDataIsEnabled = false;
                            CheckBoxSaveRAWValuesIsEnabled = false;
                            TextBlockSaveSuffixIsEnabled = false;
                        }
                        else
                        {
                            CheckBoxSaveDataIsEnabled = true;
                            if (CheckBoxSaveDataIsChecked)
                            {
                                CheckBoxSaveRAWValuesIsEnabled = true;
                                TextBlockSaveSuffixIsEnabled = true;
                            }
                        }
                    else
                    {
                        CheckBoxSaveDataIsEnabled = false;
                        CheckBoxSaveRAWValuesIsEnabled = false;
                        TextBlockSaveSuffixIsEnabled = false;
                    }
                    break;

                case "IsRunning":
                    if (PediatricSensorData.IsRunning)
                        ButtonStartStopSensorsContent = "Stop Sensors";
                    else
                        ButtonStartStopSensorsContent = "Start Sensors";
                    break;

                case "CanZeroFields":
                    App.Current.Dispatcher.Invoke(() => ButtonZeroFieldsCommand.RaiseCanExecuteChanged());
                    break;

                case "CanSendCommands":
                    RaisePropertyChanged("ButtonSendCommandsIsEnabled");
                    break;

                default:
                    break;
            }
        }

    }
}
