using LiveCharts;
using Prism.Mvvm;

namespace PediatricSoft
{
    public class PlotWindowViewModel : BindableBase
    {

        // Fields

        private PediatricSensorData PediatricSensorData;

        // Constructors

        public PlotWindowViewModel()
        {
            PediatricSensorData = PediatricSensorData.Instance;
        }

        // Properties

        public SeriesCollection SeriesCollection { get { return PediatricSensorData.SeriesCollection; } }
        
        // Methods



    }
}
