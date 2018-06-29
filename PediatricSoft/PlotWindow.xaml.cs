using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;


namespace PediatricSoft
{



    public partial class PlotWindow : Window
    {
        public PlotWindow()
        {
            InitializeComponent();
            plotBox.Series = PediatricSensorData._SeriesCollection;
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            PediatricSoftConstants.PlotWindowClosed = true;
        }
    }
}
