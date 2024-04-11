using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// {
//     "imu": {
//         "eva1": {
//             "posx": 0.000000,
// 			"posy": 0.000000,
// 			"heading": 0.000000
//         },
// 		"eva2": {
//             "posx": 0.000000,
// 			"posy": 0.000000,
// 			"heading": 0.000000
//         }
//     }
// }

namespace ARSIS.EventManager
{
    [System.Serializable]
    public class ImuEva {
        public float posx { get; set; }
        public float posy { get; set; }
        public float heading { get; set; }
    }

    [System.Serializable]
    public class ImuData
    {
        public ImuEva eva1 { get; set; }
        public ImuEva eva2 { get; set; }
    }

    [System.Serializable]
    public class ImuRoot
    {
        public ImuData imu { get; set; }
    }

    [System.Serializable]
    public class IMU : BaseArsisEvent
    {
        public ImuRoot data { get; set; }

        public override string ToString()
        {
            return "IMU event";
        }
    }
}
