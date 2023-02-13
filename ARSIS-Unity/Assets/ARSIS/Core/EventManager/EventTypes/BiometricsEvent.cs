using EventSystem;

public class BiometricsEvent : BaseArsisEvent
{
    public readonly float lat;
    public readonly float lon;

    public BiometricsEvent(float lat, float lon)
    {
        this.lat = lat;
        this.lon = lon;
    }
}

