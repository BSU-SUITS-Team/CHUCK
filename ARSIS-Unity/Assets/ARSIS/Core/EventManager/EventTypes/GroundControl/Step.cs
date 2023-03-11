using System.Collections.Generic;
using UnityEngine;

namespace EventSystem {
    public class Step : BaseArsisEvent
    {
        public readonly string type;
        public readonly string body;
        public readonly NextTask nextTask;

        public Step(string type, string body, NextTask nextTask){
            this.type = type;
            this.body = body;
            this.nextTask = nextTask;
        }
    }
}
