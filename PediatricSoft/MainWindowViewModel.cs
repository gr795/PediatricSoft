using Prism.Commands;
using Prism.Mvvm;
using System.ComponentModel;
using System.Windows.Forms;

namespace PediatricSoft
{
    // This is the view model for the Main Window
    // All Main Window interaction logic goes here
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

            // We have a method for handling certain data events
            // Assaign it on creation
            PediatricSensorData.PropertyChanged += OnPediatricSensorDataPropertyChanged;
        }

        // Properties

        // We use a string property to set the window title
        // This autmatically displays the version number of ClickOnce installations
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

        // We expose PediatricSensorData and DebugLog as properties so that we can bind to them in the MainWindow view
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

        // This property sets the text of the start/stop button
        private string buttonStartStopSensorsContent = "Start Sensors";
        public string ButtonStartStopSensorsContent
        {
            get { return buttonStartStopSensorsContent; }
            private set { buttonStartStopSensorsContent = value; RaisePropertyChanged(); }
        }

        // Methods

        // This is executed when the save data check box is toggled
        // If the save folder wasn't selected before - force the selection
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

        // Use the FolderBrowserDialog from winforms to select a folder for data save
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
                    // If we didn't select a folder - uncheck the check box
                    if (string.IsNullOrEmpty(PediatricSensorData.SaveFolder))
                    {
                        PediatricSensorData.SaveDataEnabled = false;
                    }
                }
            }
        }

        // This method signals to the window manager to open the configuration window (previously know as SendCommandsWindow)
        private void ButtonSendCommandsOnClick()
        {
            if (PediatricSensorData.DebugMode)
            {
                DebugLog.Enqueue("Main Window View Model: Send Commands Button clicked");
            }
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Publish("ShowSendCommandsWindow");
        }

        // Event handlers

        // This event handler looks for changes in the IsRunning property and sets the text of the start/stop button
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
