using System.Threading;
using System.Windows;

namespace PediatricSoft
{
    public partial class App : Application
    {
        public App()
        {
        }

        private void Application_Startup(object sender, StartupEventArgs e)
        {
            ThreadPool.SetMinThreads(512, 512);
            PediatricSoftWindowManager.Instance.Start();
        }
    }
}