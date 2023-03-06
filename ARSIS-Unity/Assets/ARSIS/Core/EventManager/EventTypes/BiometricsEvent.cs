using EventSystem;

public class BiometricsEvent : BaseArsisEvent
{
    //     return {"bpm": bpm, "o2": o2, "battery": battery}

    public readonly float bpm;
    public readonly float o2;
    public readonly float battery;

    public BiometricsEvent(float bpm, float o2, float battery)
    {
        this.bpm = bpm;
        this.o2 = o2;
        this.battery = battery;
    }
}

