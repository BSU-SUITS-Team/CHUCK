using System;
using System.Diagnostics;
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
        public string type;
        public long time;
    }
}