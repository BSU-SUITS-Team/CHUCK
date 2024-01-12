using ARSISEventSystem;

public class LocationEvent : BaseArsisEvent
{
    public readonly float latitude;
    public readonly float longitude;
    public readonly float altitude;
    public readonly float heading;

    public LocationEvent(float latitude, float longitude, float altitude, float heading)
    {
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitude = altitude;
        this.heading = heading;
    }
}

