using Prism.Mvvm;
using System;
using System.Collections.Concurrent;

namespace PediatricSoft
{
    public class DebugLog : BindableBase
    {

        // Fields
        private static readonly DebugLog instance = new DebugLog();
        private static readonly ConcurrentQueue<string> debugLogQueue = new ConcurrentQueue<string>();

        // Constructors
        static DebugLog()
        {
        }

        private DebugLog()
        {
        }

        // Properties
        public static DebugLog Instance
        {
            get
            {
                return instance;
            }
        }

        public string[] StringArray
        {
            get
            {
                string[] temp = debugLogQueue.ToArray();
                Array.Reverse(temp);
                return temp;
            }
        }

        // Methods

        public void Enqueue(string message)
        {
            debugLogQueue.Enqueue(message);
            while (debugLogQueue.Count > PediatricSoftConstants.DebugLogQueueMaxCount)
            {
                debugLogQueue.TryDequeue(out string dummy);
            }
            RaisePropertyChanged("StringArray");
        }
    }
}
