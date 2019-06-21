using Prism.Mvvm;
using System;
using System.Collections.Concurrent;

namespace PediatricSoft
{
    // This class provides a logging facility
    // We use the singleton pattern
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

        // This property provides a string array of messages
        // The order is reversed so that the latest message is the first one
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

        // Add a new message
        // We restrict the max number of messages to a value defined in the PediatricSoftConstants class
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
