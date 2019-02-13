using Prism.Events;

namespace PediatricSoft
{
    public static class PediatricSoftEventGlue
    {
        public static readonly IEventAggregator eventAggregator;

        static PediatricSoftEventGlue()
        {
            eventAggregator = new EventAggregator();
        }

    }
}