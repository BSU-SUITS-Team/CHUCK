using System.Collections.Generic;

namespace ARSISEventSystem {
    public class NavPathEvent : BaseArsisEvent
    {
        public readonly string name;
        public readonly List<NavPoint> points;
        public readonly string type;

        public NavPathEvent(string name, List<NavPoint> points, string type){
            this.name = name;
            this.points = points;
            this.type = type;
        }
    }
}
