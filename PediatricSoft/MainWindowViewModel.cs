using Prism.Commands;
using Prism.Mvvm;
using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Timers;
using System.Windows.Forms;

namespace PediatricSoft
{
    public class MainWindowViewModel : BindableBase
    {

        // Fields

        private PediatricSensorData PediatricSensorData;

        // Constructors

        public MainWindowViewModel()
        {
            PediatricSensorData = PediatricSensorData.Instance;

            ButtonScanPortsCommand = new DelegateCommand(PediatricSensorData.ScanPortsAsync, () => PediatricSensorData.CanScan);
            ButtonLockSensorsCommand = new DelegateCommand(PediatricSensorData.LockAllAsync, () => PediatricSensorData.CanLock);
            ButtonStartStopSensorsCommand = new DelegateCommand(PediatricSensorData.StartStopAsync, () => PediatricSensorData.CanStartStop);
            ButtonZeroFieldsCommand = new DelegateCommand(PediatricSensorData.ZeroFieldsAsync, () => PediatricSensorData.CanZeroFields);
            CheckBoxSaveDataCommand = new DelegateCommand(CheckBoxSaveDataOnToggle);
            ButtonSendCommandsCommand = new DelegateCommand(ButtonSendCommandsOnClick);
            ButtonChooseSaveDataFolderCommand = new DelegateCommand(ChooseSaveDataFolder);

            PediatricSensorData.PropertyChanged += OnPediatricSensorDataPropertyChanged;
        }

        // Properties
        public string WindowTitle
        {
            get
            {
                if (System.Deployment.Application.ApplicationDeployment.IsNetworkDeployed)
                    return $"PediatricSoft {System.Deployment.Application.ApplicationDeployment.CurrentDeployment.CurrentVersion.ToString(4)}";
                else
                    return "PediatricSoft";
            }
        }

        public DebugLog DebugLog { get { return DebugLog.Instance; } }

        public DelegateCommand ButtonScanPortsCommand { get; private set; }
        public DelegateCommand ButtonLockSensorsCommand { get; private set; }
        public DelegateCommand ButtonStartStopSensorsCommand { get; private set; }
        public DelegateCommand ButtonZeroFieldsCommand { get; private set; }
        public DelegateCommand CheckBoxSaveDataCommand { get; private set; }
        public DelegateCommand ButtonSendCommandsCommand { get; private set; }
        public DelegateCommand ButtonChooseSaveDataFolderCommand { get; private set; }

        public ObservableCollection<PediatricSensor> Sensors { get { return PediatricSensorData.Sensors; } }

        public bool CheckBoxDebugModeIsEnabled { get; private set; } = true;

        public bool CheckBoxDebugModeIsChecked
        {
            get { return PediatricSensorData.DebugMode; }
            set
            {
                PediatricSensorData.DebugMode = value;
                CheckBoxDebugModeIsEnabled = false;

                RaisePropertyChanged("");

                ButtonScanPortsCommand.RaiseCanExecuteChanged();
                ButtonLockSensorsCommand.RaiseCanExecuteChanged();
                ButtonStartStopSensorsCommand.RaiseCanExecuteChanged();
                ButtonZeroFieldsCommand.RaiseCanExecuteChanged();
            }
        }

        public bool ButtonSendCommandsIsEnabled
        {
            get { return PediatricSensorData.CanSendCommands; }
        }

        private bool checkBoxSaveDataIsEnabled = false;
        public bool CheckBoxSaveDataIsEnabled
        {
            get { return PediatricSensorData.DebugMode || checkBoxSaveDataIsEnabled; }
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
            get { return PediatricSensorData.DebugMode || checkBoxSaveRAWValuesIsEnabled; }
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
            get { return PediatricSensorData.DebugMode || textBlockSaveSuffixIsEnabled; }
            private set { textBlockSaveSuffixIsEnabled = value; RaisePropertyChanged(); }
        }

        public string TextBlockSaveSuffixText
        {
            get { return PediatricSensorData.SaveSuffix; }
            set { PediatricSensorData.SaveSuffix = value; }
        }

        public string TextBlockSaveFolderText
        {
            get { return String.Concat("Data Save Folder: ", PediatricSensorData.SaveFolder); }
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
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Main Window View Model: Save Data checkbox toggled");

            if (string.IsNullOrEmpty(PediatricSensorData.SaveFolder))
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
                    PediatricSensorData.SaveFolder = dialog.SelectedPath;
                    if (PediatricSensorData.DebugMode)
                    {
                        DebugLog.Enqueue($"Main Window View Model: {TextBlockSaveFolderText}");
                    }
                }
                else
                {
                    if (string.IsNullOrEmpty(PediatricSensorData.SaveFolder))
                    {
                        CheckBoxSaveDataIsChecked = false;
                    }
                }
            }
            RaisePropertyChanged("TextBlockSaveFolderText");
        }

        private void ButtonSendCommandsOnClick()
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Main Window View Model: Send Commands Button clicked");
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Publish("ShowSendCommandsWindow");
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
                    ButtonScanPortsCommand.RaiseCanExecuteChanged();
                    if (CheckBoxDebugModeIsEnabled)
                    {
                        CheckBoxDebugModeIsEnabled = false;
                        RaisePropertyChanged("CheckBoxDebugModeIsEnabled");
                    }
                    break;

                case "CanLock":
                    ButtonLockSensorsCommand.RaiseCanExecuteChanged();
                    break;

                case "CanStartStop":
                    ButtonStartStopSensorsCommand.RaiseCanExecuteChanged();
                    RaisePropertyChanged("RadioButtonDataSelectIsEnabled");
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
                    ButtonZeroFieldsCommand.RaiseCanExecuteChanged();
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
