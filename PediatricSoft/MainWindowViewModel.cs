using Prism.Commands;
using Prism.Mvvm;
using System.ComponentModel;
using System.Windows.Forms;

namespace PediatricSoft
{
    public class MainWindowViewModel : BindableBase
    {

        // Fields

        // Constructors

        public MainWindowViewModel()
        {

            ButtonScanPortsCommand = new DelegateCommand(PediatricSensorData.ScanPortsAsync);
            ButtonStandbySensorsCommand = new DelegateCommand(PediatricSensorData.StandbyAllAsync);
            ButtonLockSensorsCommand = new DelegateCommand(PediatricSensorData.LockAllAsync);
            ButtonStartStopSensorsCommand = new DelegateCommand(PediatricSensorData.StartStopAsync);
            ButtonZeroFieldsCommand = new DelegateCommand(PediatricSensorData.ZeroFieldsAsync);

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
                {
                    return $"PediatricSoft {System.Deployment.Application.ApplicationDeployment.CurrentDeployment.CurrentVersion.ToString(4)}";
                }
                else
                {
                    return "PediatricSoft";
                }
            }
        }

        public PediatricSensorData PediatricSensorData { get { return PediatricSensorData.Instance; } }
        public DebugLog DebugLog { get { return DebugLog.Instance; } }

        public DelegateCommand ButtonScanPortsCommand { get; private set; }
        public DelegateCommand ButtonStandbySensorsCommand { get; private set; }
        public DelegateCommand ButtonLockSensorsCommand { get; private set; }
        public DelegateCommand ButtonStartStopSensorsCommand { get; private set; }
        public DelegateCommand ButtonZeroFieldsCommand { get; private set; }
        public DelegateCommand CheckBoxSaveDataCommand { get; private set; }
        public DelegateCommand ButtonSendCommandsCommand { get; private set; }
        public DelegateCommand ButtonChooseSaveDataFolderCommand { get; private set; }

        private string buttonStartStopSensorsContent = "Start Sensors";
        public string ButtonStartStopSensorsContent
        {
            get { return buttonStartStopSensorsContent; }
            private set { buttonStartStopSensorsContent = value; RaisePropertyChanged(); }
        }

        // Methods

        private void CheckBoxSaveDataOnToggle()
        {
            if (PediatricSensorData.DebugMode)
            {
                DebugLog.Enqueue("Main Window View Model: Save Data checkbox toggled");
            }

            if (string.IsNullOrEmpty(PediatricSensorData.SaveFolder))
            {
                ChooseSaveDataFolder();
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
                        DebugLog.Enqueue($"Main Window View Model: Data Save Folder: {PediatricSensorData.SaveFolder}");
                    }
                }
                else
                {
                    if (string.IsNullOrEmpty(PediatricSensorData.SaveFolder))
                    {
                        PediatricSensorData.SaveDataEnabled = false;
                    }
                }
            }
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
                case "IsRunning":
                    if (PediatricSensorData.IsRunning)
                    {
                        ButtonStartStopSensorsContent = "Stop Sensors";
                    }
                    else
                    {
                        ButtonStartStopSensorsContent = "Start Sensors";
                    }
                    break;

                default:
                    break;
            }
        }

    }
}
