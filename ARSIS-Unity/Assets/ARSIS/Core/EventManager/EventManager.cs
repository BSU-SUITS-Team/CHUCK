using UnityEngine;
using System;
using System.Collections.Generic;

namespace EventSystem
{
    /// <summary>
    /// This class contains the methods that are used to
    /// add, remove, and trigger events. It is a static class
    /// so that it can be accessed from anywhere. All of the
    /// events should be of type (subclass of
    /// IArsisEvent.)
    /// </summary>
    public static class EventManager
    {
        private static Dictionary<Delegate, Action<dynamic>> wrappers = new Dictionary<Delegate, Action<dynamic>>();
        private static Dictionary<Type, Action<dynamic>> eventDictionary = new Dictionary<Type, Action<dynamic>>();
        private static List<Action<dynamic>> listenToAll = new List<Action<dynamic>>();

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

        public static void AddListenerToAll(Action<dynamic> eventFunction)
        {
            listenToAll.Add(eventFunction);
        }

        /// <summary>
        /// This adds a listener to the provided event type.
        /// </summary>
        /// <typeparam name="EventType">This is the type of the Event to add a listener to</typeparam>
        /// <param name="eventFunction">The Listener that will be called when the event is called.</param>
        /// <exception cref="Exception">That Listener is already listeneing to this event</exception>
        public static void AddListener<EventType>(Action<EventType> eventFunction) where EventType : BaseArsisEvent
        {
            if (wrappers.ContainsKey(eventFunction))
            {
                //This is just for demonstaration, we should change the error type
                throw new Exception("You can't add the same listener twice!");
            }
            wrappers.Add(eventFunction, (dynamic evt) => { eventFunction((EventType)evt); });
            AddListener(wrappers[eventFunction], typeof(EventType));
        }

        /// <summary>
        /// This removes an event listener from the provided event type.
        /// </summary>
        /// <typeparam name="EventType">The event type to remove the listener from.</typeparam>
        /// <param name="eventFunction">The lister to remove.</param>
        public static void RemoveListener<EventType>(Action<EventType> eventFunction) where EventType : BaseArsisEvent
        {
            if (eventDictionary.ContainsKey(typeof(EventType)) && wrappers.ContainsKey(eventFunction))
            {
                eventDictionary[typeof(EventType)] -= wrappers[eventFunction];
                wrappers.Remove(eventFunction);
            }
            //Might want to add an error if it does not exist or was already removed
        }

        /// <summary>
        /// This triggers events that have the same datatype as the provided data using the provided data.
        /// </summary>
        /// <param name="data">The data to pass to events</param>
        public static void Trigger(dynamic data)
        {
            if (eventDictionary.ContainsKey(data.GetType()))
            {
                eventDictionary[data.GetType()](data);
            }
            foreach(Action<dynamic> e in listenToAll){
                e(data);
            }
        }

        /// <summary>
        /// Removes all listeners from all events.
        /// </summary>
        public static void RemoveAllListeners()
        {
            eventDictionary.Clear();
            wrappers.Clear();
        }

        /// <summary>
        /// Removes all listeners from the provided event type.
        /// </summary>
        /// <typeparam name="EventType">The type to deregister events from.</typeparam>
        // public static void RemoveAllListeners<EventType>() where EventType : BaseArsisEvent
        // {
        //     if (eventDictionary.ContainsKey(typeof(EventType)))
        //     {
        //         eventDictionary.Remove(typeof(EventType));
        //     }
        // } //This need to be worked on more to clear the wrappers, before it is ready to be used.

        /// <summary>
        /// Check if a provided listener is listening to a provided event type.
        /// </summary>
        /// <typeparam name="EventType">The event type to check.</typeparam>
        /// <param name="eventFunction">The listener to check.</param>
        /// <returns>True if the listener is listening to the event type.</returns>
        public static bool IsListening<EventType>(Action<EventType> eventFunction) where EventType : BaseArsisEvent
        {
            if (wrappers.ContainsKey(eventFunction))
            {
                return Array.IndexOf(eventDictionary[typeof(EventType)].GetInvocationList(), wrappers[eventFunction]) > -1;
            }
            return false;
        }
    }
}
