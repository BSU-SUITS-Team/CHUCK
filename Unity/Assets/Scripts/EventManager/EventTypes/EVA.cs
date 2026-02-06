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
    public class MissionTimers
    {
        public bool started { get; set; }
        public bool completed { get; set; }
        public int time { get; set; }
    }

    [System.Serializable]
    public class EvaData
    {
        public bool started { get; set; }
        public bool paused { get; set; }
        public bool completed { get; set; }
        public int total_time { get; set; }
        public MissionTimers uia { get; set; }
        public MissionTimers dcu { get; set; }
        public MissionTimers rover { get; set; }
        public MissionTimers spec { get; set; }
    }

    [System.Serializable]
    public class EVA : BaseArsisEvent
    {
        public EvaData data { get; set; }

        public override string ToString()
        {
            return "EVA event";
        }
    }
}
