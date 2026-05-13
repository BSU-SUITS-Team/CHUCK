using System;

[Serializable]
public class EvaRoot
{
    public EvaTelemetry telemetry;
    public EvaStatus status;
    public EvaDcu dcu;
    public EvaError error;
    public EvaImu imu;
    public EvaUia uia;
}

[Serializable]
public class EvaTelemetry
{
    public EvaSuit eva1;
    public EvaSuit eva2;
}

[Serializable]
public class EvaSuit
{
    public float primary_battery_level; // EVA 1
    public float secondary_battery_level; // EVA 1
    public float battery_level; // EVA 2

    public float oxy_pri_storage;
    public float oxy_sec_storage;
    public float oxy_pri_pressure;
    public float oxy_sec_pressure;

    public float suit_pressure_oxy;
    public float suit_pressure_co2;
    public float suit_pressure_other;
    public float suit_pressure_total;

    public float helmet_pressure_co2;

    public float fan_pri_rpm;
    public float fan_sec_rpm;

    public float scrubber_a_co2_storage;
    public float scrubber_b_co2_storage;

    public float temperature;

    public float coolant_storage;
    public float coolant_gas_pressure;
    public float coolant_liquid_pressure;

    public float heart_rate;
    public float oxy_consumption;
    public float co2_production;
    public float eva_elapsed_time;
}

[Serializable]
public class EvaStatus
{
    public bool started;
}

[Serializable]
public class EvaDcu
{
    public Eva1Dcu eva1;
    public Eva2Dcu eva2;
}

[Serializable]
public class Eva1Dcu
{
    public bool oxy;
    public bool fan;
    public bool pump;
    public bool co2;
    public Eva1Batt batt;
}

[Serializable]
public class Eva1Batt
{
    public bool lu;
    public bool ps;
}

[Serializable]
public class Eva2Dcu
{
    public bool batt;
    public bool oxy;
    public bool comm;
    public bool fan;
    public bool pump;
    public bool co2;
}

[Serializable]
public class EvaError
{
    public bool fan_error;
    public bool oxy_error;
    public bool power_error;
    public bool scrubber_error;
}

[Serializable]
public class EvaImu
{
    public EvaImuSuit eva1;
    public EvaImuSuit eva2;
}

[Serializable]
public class EvaImuSuit
{
    public float posx;
    public float posy;
    public float heading;
}

[Serializable]
public class EvaUia
{
    public bool eva1_power;
    public bool eva1_oxy;
    public bool eva1_water_supply;
    public bool eva1_water_waste;

    public bool eva2_power;
    public bool eva2_oxy;
    public bool eva2_water_supply;
    public bool eva2_water_waste;

    public bool oxy_vent;
    public bool depress;
}
