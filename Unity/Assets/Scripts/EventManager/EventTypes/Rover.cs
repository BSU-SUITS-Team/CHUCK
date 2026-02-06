using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// {
//     "rover": {
//      "posx": 0.000000,
// 		"posy": 0.000000,
// 		"qr_id": 0
//     }
// }

namespace ARSIS.EventManager
{

    [System.Serializable]
    public class Data
    {
        public float posx { get; set; }
        public float posy { get; set; }
        public int qr_id { get; set; }
    }

    [System.Serializable]
    public class Root
    {
        public Data rover { get; set; }
    }

    [System.Serializable]
    public class Rover : BaseArsisEvent
    {
        public Root data { get; set; }

        public override string ToString()
        {
            return "Rover event";
        }
    }
}
