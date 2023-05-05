using System.Collections.Generic;

namespace ARSISEventSystem{
    public class ProcedureGet : BaseArsisEvent
    {
        public string navigationPathName;
        public ProcedureGet(string navigationPathName){
            this.navigationPathName = navigationPathName;
        }
    }
}
