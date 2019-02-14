using Prism.Events;
using System.Threading;
using System.Windows;

namespace PediatricSoft
{
    public partial class App : Application
    {
        private IEventAggregator eventAggregator;
        private PediatricSensorData PediatricSensorData;
        private PediatricSoftWindowManager PediatricSoftWindowManager;

        public App()
        {
            ThreadPool.SetMinThreads(128, 128);
            eventAggregator = PediatricSoftEventGlue.eventAggregator;
            PediatricSensorData = PediatricSensorData.Instance;
            PediatricSoftWindowManager = PediatricSoftWindowManager.Instance;
        }
    }
}