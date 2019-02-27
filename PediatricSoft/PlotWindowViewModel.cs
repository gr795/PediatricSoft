using LiveCharts;
using LiveCharts.Helpers;
using LiveCharts.Wpf;
using Prism.Mvvm;
using Prism.Events;
using System;
using System.Diagnostics;
using System.Linq;
using System.Timers;
using System.Windows.Media;
using System.ComponentModel;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace PediatricSoft
{
    public class PlotWindowViewModel : BindableBase, IDisposable
    {

        // Fields

        private readonly PediatricSensorData PediatricSensorData = PediatricSensorData.Instance;
        private readonly SubscriptionToken SubscriptionTokenEventDataLayer;
        private readonly SubscriptionToken SubscriptionTokenEventUILayer;

        // Constructors

        public PlotWindowViewModel()
        {
            SubscriptionTokenEventDataLayer = PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Subscribe(DataLayerEventHandler);
            SubscriptionTokenEventUILayer = PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Subscribe(WindowManagerEventHandler);

            SeriesCollection = new SeriesCollection();
        }

        // Properties

        public SeriesCollection SeriesCollection { get; private set; }

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
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Plot Window View Model: UpdateSeriesCollection");

            SeriesCollection = new SeriesCollection();

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
                }
            }

            RaisePropertyChanged("SeriesCollection");
        }

        public void Dispose()
        {
            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Plot window view model: calling Dispose()");

            PediatricSoftEventGlue.eventAggregator.GetEvent<EventDataLayer>().Unsubscribe(SubscriptionTokenEventDataLayer);
            PediatricSoftEventGlue.eventAggregator.GetEvent<EventUILayer>().Unsubscribe(SubscriptionTokenEventUILayer);

            Debug.WriteLineIf(PediatricSoftConstants.IsDebugEnabled, "Plot window view model: Dispose() done");
        }

        // Event handlers

    }
}
