using Prism.Commands;
using Prism.Mvvm;
using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Timers;
using System.Windows.Forms;

namespace PediatricSoft
{
    public class MainWindowViewModel : BindableBase
    {

        // Fields

        private PediatricSensorData PediatricSensorData;
        private System.Timers.Timer uiUpdateTimer;

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

            uiUpdateTimer = new System.Timers.Timer(PediatricSoftConstants.UIUpdateInterval);
            uiUpdateTimer.Elapsed += OnUIUpdateTimerEvent;
            uiUpdateTimer.Enabled = true;
        }

        // Properties

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

        public bool RadioButtonDataSelectIsEnabled
        {
            get { return PediatricSensorData.DebugMode || (!PediatricSensorData.IsRunning && PediatricSensorData.CanStartStop); }
        }

        public string[] DebugLog
        {
            get
            {
                string[] temp = PediatricSensorData.DebugLogQueue.ToArray();
                Array.Reverse(temp);
                return temp;
            }
        }

        // Methods

        private void CheckBoxSaveDataOnToggle()
        {
            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue("Main Window View Model: Save Data checkbox toggled");

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
                    if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue($"Main Window View Model: Save Data Folder: {TextBlockSaveFolderText}");
                }
                else
                    if (string.IsNullOrEmpty(TextBlockSaveFolderText)) CheckBoxSaveDataIsChecked = false;
            }
        }

        private void ButtonSendCommandsOnClick()
        {
            if (PediatricSensorData.DebugMode) PediatricSensorData.DebugLogQueue.Enqueue("Main Window View Model: Send Commands Button clicked");
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Publish("ShowSendCommandsWindow");
        }

        private void RadioButtonsRaisePropertyChanged()
        {
            RaisePropertyChanged("RadioButtonDataSelectADCIsChecked");
            RaisePropertyChanged("RadioButtonDataSelectOpenLoopIsChecked");
            RaisePropertyChanged("RadioButtonDataSelectClosedLoopIsChecked");
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

        private void OnUIUpdateTimerEvent(Object source, ElapsedEventArgs e)
        {
            while (PediatricSensorData.DebugLogQueue.Count > PediatricSoftConstants.DebugLogQueueMaxCount)
                PediatricSensorData.DebugLogQueue.TryDequeue(out string dummy);
            RaisePropertyChanged("DebugLog");
        }

    }
}
