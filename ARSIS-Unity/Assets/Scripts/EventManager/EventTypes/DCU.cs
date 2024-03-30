using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// {
// 	"dcu": {
// 		"eva1": {
// 			"batt": true,
// 			"oxy": true,
// 			"comm": true,
// 			"fan": true,
// 			"pump": true,
// 			"co2": true
// 		},
// 		"eva2": {
// 			"batt": true,
// 			"oxy": true,
// 			"comm": true,
// 			"fan": true,
// 			"pump": true,
// 			"co2": true
// 		}
// 	}
// }

namespace ARSIS.EventManager
{
    [System.Serializable]
    public class DcuEva
    {
        public bool batt { get; set; }
        public bool oxy { get; set; }
        public bool comm { get; set; }
        public bool fan { get; set; }
        public bool pump { get; set; }
        public bool co2 { get; set; }
    }

    [System.Serializable]
    public class DcuData
    {
        public DcuEva eva1 { get; set; }
        public DcuEva eva2 { get; set; }
    }

    [System.Serializable]
    public class DcuRoot
    {
        public DcuData dcu { get; set; }
    }

    [System.Serializable]
    public class DCU : BaseArsisEvent
    {
        public DcuRoot data { get; set; }

        public override string ToString()
        {
            return "DCU event";
        }
    }
}
