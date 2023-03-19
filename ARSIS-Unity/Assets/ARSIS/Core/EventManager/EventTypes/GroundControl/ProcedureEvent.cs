using System.Collections.Generic;
using UnityEngine;

namespace EventSystem {
    public class ProcedureEvent : BaseArsisEvent
    {
        public readonly string name;
        public readonly string summary;
        public readonly List<Task> taskList;

        public ProcedureEvent(string name, string summary, List<Task> taskList){
            this.name = name;
            this.summary = summary;
            this.taskList = taskList;
        }
    }
}
