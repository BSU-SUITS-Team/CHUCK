using System.Collections.Generic;

namespace ARSISEventSystem{
    public class ProcedureDictionary : BaseArsisEvent
    {
        public Dictionary<string, ProcedureEvent> procedureDictionary;
        public ProcedureDictionary(Dictionary<string, ProcedureEvent> procedureDictionary){
            this.procedureDictionary = procedureDictionary;
        }
    }
}
