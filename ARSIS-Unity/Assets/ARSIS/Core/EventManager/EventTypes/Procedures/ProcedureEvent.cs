using System.Collections.Generic;
using UnityEngine;

namespace ARSISEventSystem {
    public class ProcedureEvent : BaseArsisEvent
    {
        public readonly string name;
        public readonly string summary;
        public readonly List<ARSISTask> taskList;

        public ProcedureEvent(string name, string summary, List<ARSISTask> taskList){
            this.name = name;
            this.summary = summary;
            this.taskList = taskList;
        }
    }
}
