namespace ARSISEventSystem {
    public class NavPoint : BaseArsisEvent
    {
        public readonly string name;
        public readonly float lat;
        public readonly float lon;
        public readonly float altitude;


        public NavPoint(string name, float lat, float lon, float altitude){
            this.name = name;
            this.lat = lat;
            this.lon = lon;
            this.altitude = altitude;
        }
    }
}
