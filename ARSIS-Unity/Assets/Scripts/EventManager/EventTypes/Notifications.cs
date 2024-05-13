using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.EventManager
{

    [System.Serializable]
    public class NotificationsData
    {
        public ImuEva eva1 { get; set; }
        public ImuEva eva2 { get; set; }
    }

    [System.Serializable]
    public class NotificationsRoot
    {
        public ImuData imu { get; set; }
    }

    [System.Serializable]
    public class Notifications : BaseArsisEvent
    {
        public ImuRoot data { get; set; }

        public override string ToString()
        {
            return "IMU event";
        }
    }
}
