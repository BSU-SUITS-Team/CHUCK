using System; 
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// {
//  "type":"pins",
//  "time":1715632029270962048,
//  "data": {
//     "type":"Point",
//     "properties": {
//         "name":"",
//         "id":"884fbd3e-f432-4873-a151-86519c33c4aa",
//         "x":481,
//         "y":403,
//         "northing": int,
//         "easting": int
//     },
//     "coordinates":[-95.08192908,29.56532568]
//  },
//  "label":"884fbd3e-f432-4873-a151-86519c33c4aa"
// }

namespace ARSIS.EventManager
{
    [System.Serializable]
    public class PinsData
    {
        public string type { get; set; }
        public PinsProperties properties { get; set; }
        public List<double> coordinates { get; set; }
    }

    [System.Serializable]
    public class PinsProperties
    {
        public string name { get; set; }
        public string id { get; set; }
        public int x { get; set; }
        public int y { get; set; }
        public int northing { get; set; }
        public int easting { get; set; }
    }

    [System.Serializable]
    public class PinsRoot
    {
        public string type { get; set; }
        public long time { get; set; }
        public PinsData pins { get; set; }
        public string label { get; set; }
    }

    [System.Serializable]
    public class Pins : BaseArsisEvent
    {
        public PinsData data { get; set; }

        public override string ToString()
        {
            
           // return "Pins event"; 
           return string.Join(
                Environment.NewLine,
                "properties: ",
                //$"\tname: {this.data.pinsProperties.name}",
                //$"\tname: {this.data.pinsProperties.id}",
                $"\tname: {data.properties.name}", //UTM
                $"\tx: {data.properties.x}",// UTM
                $"\ty: {data.properties.y}" //UTM
                //$"\tname: {this.data.coordinates}"
                //$"\tname: {this.data.coordinates}"
                // $"\tid: {this.data.id}",
            );
        }
    }
}
