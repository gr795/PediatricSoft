using System.Windows;

namespace PediatricSoft
{
    public partial class MainWindow : Window
    {
        private MainWindowViewModel ViewModel;

        public MainWindow()
        {
            InitializeComponent();
            ViewModel = DataContext as MainWindowViewModel;
            Closing += ViewModel.WindowMainWindowOnClosing;
        }
    }
}