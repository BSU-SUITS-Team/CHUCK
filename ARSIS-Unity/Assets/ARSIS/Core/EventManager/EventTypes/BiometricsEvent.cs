using ARSISEventSystem;

public class BiometricsEvent : BaseArsisEvent
{
    //     return {"heartrate": heartrate, "o2": o2, "battery": battery}

    public readonly int heartrate;
    public readonly int o2;
    public readonly int battery;
    public readonly int fan;
    public readonly bool vent;
    public readonly int co2;
    public readonly bool sop;
    public readonly int suitPressure;

    /* fan: int */
    /* vent: bool */
    /* co2: int */
    /* sop: bool */
    /* suitPressure: int */

    public BiometricsEvent(int heartrate, int o2, int battery, int fan, bool vent, int co2, bool sop, int suitPressure)
    {
        this.heartrate = heartrate;
        this.o2 = o2;
        this.battery = battery;
        this.fan = fan;
        this.vent = vent;
        this.co2 = co2;
        this.sop = sop;
        this.suitPressure = suitPressure;
    }
}

