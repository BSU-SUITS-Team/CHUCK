using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace ARSIS.EventManager
{
    /// <summary>
    /// This is an interface that is used to ensure a level of
    /// uniformity with what events can do.
    /// </summary>
    [System.Serializable]
    public class BaseArsisEvent
    {
        public static readonly Dictionary<string, Type> EventTypes = new() {
            { "telemetry", typeof(Telemetry) },
        };

        public string type;
        public string label;

        public static Type GetType(string type) {
            try
            {
                return EventTypes[type];
            }
            catch (KeyNotFoundException)
            {
                return typeof(BaseArsisEvent);
            }
        }
    }
}
