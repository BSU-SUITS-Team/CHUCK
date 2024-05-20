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
            { "dcu", typeof(DCU) },
            { "eva", typeof(EVA) },
            { "imu", typeof(IMU) },
            { "uia", typeof(UIA ) },
            { "rover", typeof(Rover) },
            { "procedure", typeof(Procedure) },
            { "spec", typeof(Spectrometry) },
            { "notification", typeof(Notifications) },
            { "pins", typeof(Pins) },
        };

        public string type;
        public long time;
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

        public override string ToString()
        {
            return type + " event";
        }
    }
}
