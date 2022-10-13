using System;
using System.Collections.Generic;

namespace EventManagerSystem
{
    public static class EventManager
    {
        private static Dictionary<Delegate, Action<dynamic>> wrappers = new Dictionary<Delegate, Action<dynamic>>();
        private static Dictionary<Type, Action<dynamic>> eventDictionary = new Dictionary<Type, Action<dynamic>>();

        private static void AddListener(Action<dynamic> eventFunction, Type type)
        {
            if (eventDictionary.ContainsKey(type))
            {
                eventDictionary[type] += eventFunction;
            }
            else
            {
                eventDictionary.Add(type, eventFunction);
            }
        }

        public static void AddListener<EventType>(Action<EventType> eventFunction) where EventType : IArsisEvent {
            if (wrappers.ContainsKey(eventFunction)) {
                //This is just for demonstaration, we should change the error type
                throw new Exception("You can't add the same listener twice!");
            }
            wrappers.Add(eventFunction, (dynamic evt) => {eventFunction((EventType)evt); });
            AddListener(wrappers[eventFunction], typeof(EventType));
        }

        public static void RemoveListener<EventType>(Action<EventType> eventFunction) where EventType : IArsisEvent
        {
            if (eventDictionary.ContainsKey(typeof(EventType)))
            {
                eventDictionary[typeof(EventType)] -= wrappers[eventFunction];
                wrappers.Remove(eventFunction);
            }
            //Might want to add an error if it does not exist or was already removed
        }

        public static void Trigger(dynamic data)
        {
            if (eventDictionary.ContainsKey(data.GetType()))
            {
                eventDictionary[data.GetType()](data);
            }
        }
    }    
}
