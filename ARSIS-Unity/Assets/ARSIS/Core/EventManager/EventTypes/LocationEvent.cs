using EventSystem;

public class LocationEvent : BaseArsisEvent
{
    public readonly float lat;
    public readonly float lon;

    public LocationEvent(float lat, float lon)
    {
        this.lat = lat;
        this.lon = lon;
    }
}

