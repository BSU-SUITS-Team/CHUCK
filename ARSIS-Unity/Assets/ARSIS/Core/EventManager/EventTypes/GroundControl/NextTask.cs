using System.Collections.Generic;
using UnityEngine;

namespace EventSystem {
    public class NextTask : BaseArsisEvent
    {
        public readonly string procedure;
        public readonly int task;

        public NextTask(string procedure, int task){
            this.procedure = procedure;
            this.task = task;
        }
    }
}
