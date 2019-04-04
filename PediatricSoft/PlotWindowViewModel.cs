using LiveCharts;
using LiveCharts.Configurations;
using LiveCharts.Wpf;
using Prism.Commands;
using Prism.Events;
using Prism.Mvvm;
using System;
using System.Windows.Media;

namespace PediatricSoft
{
    public class PlotWindowViewModel : BindableBase, IDisposable
    {

        // Fields

        private readonly PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;
        private DebugLog DebugLog = DebugLog.Instance;
        private readonly SubscriptionToken SubscriptionTokenEventDataLayer;
        private readonly SubscriptionToken SubscriptionTokenEventUILayer;
        private readonly CartesianMapper<XYPoint> mapper = Mappers.Xy<XYPoint>().X(v => v.X).Y(v => v.LogYAvg);

        // Constructors

        public PlotWindowViewModel()
        {
            SubscriptionTokenEventDataLayer = PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Subscribe(DataLayerEventHandler);
            SubscriptionTokenEventUILayer = PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Subscribe(WindowManagerEventHandler);

            SeriesCollection = new SeriesCollection();
            SeriesCollectionFFT = new SeriesCollection(mapper);

            Formatter = value => Math.Pow(Base, value).ToString("+0.00E+00;-0.00E+00");

            ButtonClearFFTCommand = new DelegateCommand(PediatricSensorData.ClearFFTAll);
        }

        // Properties

        public DelegateCommand ButtonClearFFTCommand { get; private set; }

        public SeriesCollection SeriesCollection { get; private set; }
        public SeriesCollection SeriesCollectionFFT { get; private set; }

        public Func<double, string> Formatter { get; private set; }
        public double Base { get { return XYPoint.Base; } }

        // Methods

        private void DataLayerEventHandler(string eventString)
        {
            switch (eventString)
            {
                case "UpdateSeriesCollection":
                    UpdateSeriesCollection();
                    break;

                default:
                    break;
            }
        }

        private void WindowManagerEventHandler(string eventString)
        {
            if (eventString == "ClosePlotWindow") Dispose();
        }

        private void UpdateSeriesCollection()
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Plot Window View Model: UpdateSeriesCollection");

            SeriesCollection = new SeriesCollection();
            SeriesCollectionFFT = new SeriesCollection(mapper);

            foreach (PediatricSensor sensor in PediatricSensorData.Sensors)
            {
                if (sensor.IsPlotted)
                {
                    SeriesCollection.Add(new LineSeries
                    {
                        Values = sensor.ChartValues,
                        Fill = Brushes.Transparent,
                        PointGeometry = DefaultGeometries.None
                    });

                    SeriesCollectionFFT.Add(new LineSeries
                    {
                        Values = sensor.ChartValuesFFT,
                        Fill = Brushes.Transparent,
                        PointGeometry = DefaultGeometries.None
                    });
                }
            }

            RaisePropertyChanged("SeriesCollection");
            RaisePropertyChanged("SeriesCollectionFFT");
        }

        public void Dispose()
        {
            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Plot window view model: calling Dispose()");

            PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Unsubscribe(SubscriptionTokenEventDataLayer);
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Unsubscribe(SubscriptionTokenEventUILayer);

            if (PediatricSensorData.DebugMode) DebugLog.Enqueue("Plot window view model: Dispose() done");
        }

        // Event handlers

    }
}
