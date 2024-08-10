using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// {
//     "uia": {
//      "eva1_power":        true,
// 		"eva1_oxy":          true,
// 		"eva1_water_supply": false,
// 		"eva1_water_waste":  false,
// 		"eva2_power":        false,
// 		"eva2_oxy":          false,
// 		"eva2_water_supply": true,
// 		"eva2_water_waste":  false,
// 		"oxy_vent":          false,
// 		"depress":           false
//     }
// }

namespace ARSIS.EventManager
{

    [System.Serializable]
    public class UiaData
    {
        public bool eva1_power { get; set; }
        public bool eva1_oxy { get; set; }
        public bool eva1_water_supply { get; set; }
        public bool eva1_water_waste { get; set; }
        public bool eva2_power { get; set; }
        public bool eva2_oxy { get; set; }
        public bool eva2_water_supply { get; set; }
        public bool eva2_water_waste { get; set; }
        public bool oxy_vent { get; set; }
        public bool depress { get; set; }
    }

    [System.Serializable]
    public class UiaRoot
    {
        public UiaData uia { get; set; }
    }

    [System.Serializable]
    public class UIA : BaseArsisEvent
    {
        public UiaRoot data { get; set; }

        public override string ToString()
        {
            return "UIA event";
        }
    }
}
