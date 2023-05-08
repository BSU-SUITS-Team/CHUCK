using System.Collections.Generic;

namespace ARSISEventSystem{
    public class NavigationGet : BaseArsisEvent
    {
        public string navigationPathName;
        public NavigationGet(string navigationPathName){
            this.navigationPathName = navigationPathName;
        }
    }
}
