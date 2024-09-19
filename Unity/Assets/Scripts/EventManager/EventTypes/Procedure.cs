using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.EventManager
{
    [System.Serializable]
    public class NextProcedure
    {
        public string procedure { get; set; }
        public int task { get; set; }
    }

    [System.Serializable]
    public class ProcedureStep
    {
        public string type { get; set; }
        public string body { get; set; }
        public List<NextProcedure> nextTask { get; set; }
    }

    [System.Serializable]
    public class ProcedureTask
    {
        public string description {  get; set; }
        public string name { get; set; }
        public List<ProcedureStep> steps { get; set; }
    }

    [System.Serializable]
    public class ProcedureRoot
    {
        public string category { get; set; }
        public string description { get; set; }
        public string duration { get; set; }
        public string name { get; set; }
        public List<ProcedureTask> tasks { get; set; }
    }

    [System.Serializable]
    public class Procedure : BaseArsisEvent
    {
        public ProcedureRoot data { get; set; }

        public override string ToString()
        {
            return "Procedure event";
        }
    }
}
