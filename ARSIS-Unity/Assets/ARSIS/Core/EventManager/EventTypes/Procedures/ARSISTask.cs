using System.Collections.Generic;
using UnityEngine;

namespace ARSISEventSystem {
    public class ARSISTask : BaseArsisEvent
    {
        public readonly string name;
        public readonly string summary;
        public readonly List<Step> stepList;

        public ARSISTask(string name, string summary, List<Step> stepList){
            this.name = name;
            this.summary = summary;
            this.stepList = stepList;
        }
    }
}
