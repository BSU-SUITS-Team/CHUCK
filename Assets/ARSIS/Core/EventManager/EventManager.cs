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
            MethodInfo delagateInfo = eventFunction.Method;
            Debug.Log((delagateInfo.GetParameters()[0].ParameterType.GetConstructor(BindingFlags.Public | BindingFlags.Instance, null, CallingConventions.HasThis, new Type[0], null)).ToString());
            string id = ((IArsisEvent)(delagateInfo.GetParameters()[0].ParameterType.GetConstructors()[0].Invoke(null))).eventId;
            Debug.Log(id);
            // if (eventDictionary.ContainsKey(id))
            // {
            //     if (type is not null && eventDictionary[id].type != type)
            //     {
            //         //This should never happen now
            //         throw new Exception("Event type mismatch. Event tried to register to: " + id + " with argument type: " + type + " but event already exists with type: " + eventDictionary[id].type);
            //     }
            //     eventDictionary[id].actions.Add(eventFunction);
            // }
            // else
            // {
            //     eventDictionary.Add(id, new EventData(eventFunction, type));
            // }
        }

        public static void AddListener(Action<IArsisEvent> eventFunction) {
            AddListener(eventFunction, eventFunction.GetType());
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
