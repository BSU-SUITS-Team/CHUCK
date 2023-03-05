using System.Collections.Generic;
using UnityEngine;

namespace EventSystem {
    public class Step : BaseArsisEvent
    {
        public readonly string type;
        public readonly string body;

        public Step(string type, string body){
            this.type = type;
            this.body = body;
        }
    }
}
