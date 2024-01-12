using ARSISEventSystem;
using System.Collections.Generic;

public class BiometricsEvent : BaseArsisEvent
{
    //     return {"heartrate": heartrate, "o2": o2, "battery": battery}

    public readonly int heartrate;
    public readonly int[] heartrateLimit = {80, 100};
    public readonly string heartrateName = "Heart Rate";
    public (int, int[], string) heartrateTuple;

    public readonly int o2;
    public readonly int[] o2Limit = {750, 950};
    public readonly string o2Name = "Oxygen";
    public (int, int[], string) o2Tuple;

    public readonly int battery;
    public readonly int[] batteryLimit = {0, 30};
    public readonly string batteryName = "Battery";
    public (int, int[], string) batteryTuple;

    public readonly int fan;
    public readonly int[] fanLimit = {10000, 40000};
    public readonly string fanName = "fan";
    public (int, int[], string) fanTuple;

    public readonly bool vent;
    public readonly bool ventLimit = false;
    public readonly string ventName = "Vent";
    public (bool, bool, string) ventTuple;

    public readonly int co2;
    public readonly int[] co2Limit = {0, 10};
    public readonly string co2Name = "Carbon Dioxide";
    public (int, int[], string) co2Tuple;

    public readonly bool sop;
    public readonly bool sopLimit = false;
    public readonly string sopName = "SOP";
    public (bool, bool, string) sopTuple;

    public readonly int suitPressure;
    public readonly int[] suitPressureLimit = {2, 4};
    public readonly string suitPressureName = "Suit Pressure";
    public (int, int[], string) suitPressureTuple;

    public List<(int, int[], string)> intCheckList;
    public List<(bool, bool, string)> boolCheckList;

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
        intCheckList = new List<(int, int[], string)>();
        boolCheckList = new List<(bool, bool, string)>();
        createChecks();
    }

    public void createChecks(){
        heartrateTuple = (heartrate, heartrateLimit, heartrateName);
        o2Tuple = (o2, o2Limit, o2Name);
        batteryTuple = (battery, batteryLimit, batteryName);
        fanTuple = (fan, fanLimit, fanName);
        co2Tuple = (co2, co2Limit, co2Name);
        suitPressureTuple = (suitPressure, suitPressureLimit, suitPressureName);
        intCheckList.Clear();
        intCheckList.Add(heartrateTuple);
        intCheckList.Add(o2Tuple);
        intCheckList.Add(batteryTuple);
        intCheckList.Add(fanTuple);
        intCheckList.Add(co2Tuple);
        intCheckList.Add(suitPressureTuple);
        ventTuple = (vent, ventLimit, ventName);
        sopTuple = (sop, sopLimit, sopName);
        boolCheckList.Clear();
        boolCheckList.Add(ventTuple);
        boolCheckList.Add(sopTuple);
    }
}

