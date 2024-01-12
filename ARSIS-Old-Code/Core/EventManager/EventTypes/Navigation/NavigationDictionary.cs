using System.Collections.Generic;

namespace ARSISEventSystem{
    public class NavigationDictionary : BaseArsisEvent
    {
        public Dictionary<string, NavigationEvent> navigationDictionary;
        public NavigationDictionary(Dictionary<string, NavigationEvent> navigationDictionary){
            this.navigationDictionary = navigationDictionary;
        }
    }
}
