using System.Windows;
using System.ComponentModel;

namespace PediatricSoft
{
    public partial class MainWindow : Window
    {
        private MainWindowViewModel ViewModel;

        public MainWindow()
        {
            InitializeComponent();
            ViewModel = DataContext as MainWindowViewModel;
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            ViewModel.WindowMainWindowOnClosing();
        }
    }
}