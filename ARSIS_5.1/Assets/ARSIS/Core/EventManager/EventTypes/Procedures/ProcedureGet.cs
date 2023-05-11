using System.Collections.Generic;

namespace ARSISEventSystem{
    public class ProcedureGet : BaseArsisEvent
    {
        public string procedureName;
        public ProcedureGet(string procedureName){
            this.procedureName = procedureName;
        }
    }
}
