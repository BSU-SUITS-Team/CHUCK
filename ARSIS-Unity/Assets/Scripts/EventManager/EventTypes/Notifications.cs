using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.EventManager
{

    [System.Serializable]
    public class NotificationsData
    {
        public string content ;
        public int severity;
        public int time;
    }

    [System.Serializable]
    public class NotificationsRoot
    {
        public NotificationsData notification;
    }

    [System.Serializable]
    public class Notifications : BaseArsisEvent
    {
        public NotificationsData data { get; set; }

        public override string ToString()
        {
            return "Notification event";
        }
    }
}
