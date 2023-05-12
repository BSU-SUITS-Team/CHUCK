using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SimulationControls : ISerializationCallbackReceiver
{
    public bool ok;
    public string? err;
    public Controls? controls;
    public Failures? failures;

    public void OnBeforeSerialize() { }
    public void OnAfterDeserialize() { }
}

[System.Serializable]
public class Controls
{
    public bool fan_switch;
    public bool suit_power;
    public bool o2_switch;
    public bool aux;
    public bool rca;
    public bool pump;
}

[System.Serializable]
public class Failures
{
    public bool o2_error;
    public bool pump_error;
    public bool power_error;
    public bool fan_error;
}
