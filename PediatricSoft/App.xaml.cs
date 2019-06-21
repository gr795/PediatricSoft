using System.Threading;
using System.Windows;

namespace PediatricSoft
{
    public partial class App : Application
    {
        public App()
        {
        }

        // This method is executed on application startup
        private void Application_Startup(object sender, StartupEventArgs e)
        {
            // Set the minimum number of threads the thread pool will create before throttling
            // This is done to avoid throttling with large number of sensors
            ThreadPool.SetMinThreads(512, 512);

            // Start the window manager
            // It handles all activities from this point
            PediatricSoftWindowManager.Instance.Start();
        }
    }
}