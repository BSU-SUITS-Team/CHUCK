using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace EventManager
{
    public static class EventManager
    {
        private struct EventData
        {
            public List<Action<IArsisEvent>> actions;
            public Type type; 

            public EventData(Action<IArsisEvent> action, Type type)
            {
                actions = new List<Action<IArsisEvent>>();
                actions.Add(action);
                this.type = type;
            }
        }
        private static Dictionary<string, EventData> eventDictionary = new Dictionary<string, EventData>();

        private static void AddListener(Action<IArsisEvent> eventFunction, Type type)
        {
            if (eventDictionary.ContainsKey(type.Name))
            {
                //This should never happen now
                if (type is not null && eventDictionary[type.Name].type != type)
                {
                    throw new Exception("Event type mismatch. Event tried to register to: " + type.Name + " with argument type: " + type + " but event already exists with type: " + eventDictionary[type.Name].type);
                }
                eventDictionary[type.Name].actions.Add(eventFunction);
            }
            else
            {
                eventDictionary.Add(type.Name, new EventData(eventFunction, type));
            }
        }

        public static void AddListener(Action<TestArsisIntEvent> eventFunction) {
            AddListener((IArsisEvent evt) => {eventFunction((TestArsisIntEvent)evt); }, typeof(TestArsisIntEvent));
        }

        private static void RemoveListener(string eventName, Action<dynamic> eventFunction)
        {
            if (eventDictionary.ContainsKey(eventName))
            {
                eventDictionary[eventName].actions.Remove(eventFunction);
            }
        }

        // #region RemoveTyping
        // public static void RemoveListener(string eventName, Action eventFunction) {
        //     RemoveListener(eventName, _wrappers[eventFunction]);
        // }
        // public static void RemoveListener(string eventName, Action<int> eventFunction) {
        //     RemoveListener(eventName, _wrappers[eventFunction]);
        // }
        // public static void RemoveListener(string eventName, Action<Vector3> eventFunction) {
        //     RemoveListener(eventName, _wrappers[eventFunction]);
        // }
        // public static void RemoveListener(string eventName, Action<string> eventFunction) {
        //     RemoveListener(eventName, _wrappers[eventFunction]);
        // }
        // #endregion

        // public static void Trigger(string eventName, dynamic data)
        // {
        //     if (eventDictionary.ContainsKey(eventName))
        //     {
        //         if (data is not null && eventDictionary[eventName].type != data.GetType())
        //         {
        //             throw new Exception("Event type mismatch. Tried to trigger event: " + eventName + " with type: " + data.GetType() + " but event is of type: " + eventDictionary[eventName].type);
        //         }
        //         foreach (Action<dynamic> eventFunction in eventDictionary[eventName].actions)
        //         {
        //             eventFunction(data);
        //         }
        //     }
        // }
        // public static void Trigger(string eventName)
        // {
        //     Trigger(eventName, null);
        // }
    }    
}
