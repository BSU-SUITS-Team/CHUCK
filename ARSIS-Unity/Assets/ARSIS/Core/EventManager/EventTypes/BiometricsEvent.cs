using ARSISEventSystem;

public class BiometricsEvent : BaseArsisEvent
{
    //     return {"heartrate": heartrate, "o2": o2, "battery": battery}

    public readonly float heartrate;
    public readonly float o2;
    public readonly float battery;

    public BiometricsEvent(float heartrate, float o2, float battery)
    {
        this.heartrate = heartrate;
        this.o2 = o2;
        this.battery = battery;
    }
}

