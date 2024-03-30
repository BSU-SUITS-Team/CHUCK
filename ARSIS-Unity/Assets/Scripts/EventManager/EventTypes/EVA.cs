using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// {
// 	"eva": {
// 		"started": false,
// 		"paused": false,
// 		"completed": false,
// 		"total_time": 0,
// 		"uia": {
// 			"started": false,
// 			"completed": false,
// 			"time": 0
// 		},
// 		"dcu": {
// 			"started": false,
// 			"completed": false,
// 			"time": 0
// 		},
// 		"rover": {
// 			"started": false,
// 			"completed": false,
// 			"time": 0
// 		},
// 		"spec": {
// 			"started": false,
// 			"completed": false,
// 			"time": 0
// 		}
// 	}
// }

namespace ARSIS.EventManager
{
    [System.Serializable]
    public class Eva
    {
        public bool started { get; set; }
        public bool completed { get; set; }
        public int time { get; set; }
    }

    [System.Serializable]
    public class Data
    {
        public bool started { get; set; }
        public bool paused { get; set; }
        public bool completed { get; set; }
        public int total_time { get; set; }
        public Eva uia { get; set; }
        public Eva dcu { get; set; }
        public Eva rover { get; set; }
        public Eva spec { get; set; }
    }

    [System.Serializable]
    public class Root
    {
        public Data eva { get; set; }
    }

    [System.Serializable]
    public class IMU : BaseArsisEvent
    {
        public Root data { get; set; }

        public override string ToString()
        {
            return "IMU event";
        }
    }
}
