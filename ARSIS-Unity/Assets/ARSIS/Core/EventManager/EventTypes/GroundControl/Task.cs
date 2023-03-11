using System.Collections.Generic;
using UnityEngine;

namespace EventSystem {
    public class Task : BaseArsisEvent
    {
        public readonly string name;
        public readonly string summary;
        public readonly List<Step> stepList;

        public Task(string name, string summary, List<Step> stepList){
            this.name = name;
            this.summary = summary;
            this.stepList = stepList;
        }
    }
}
