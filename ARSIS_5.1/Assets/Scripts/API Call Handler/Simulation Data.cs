using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SimulationData : ISerializationCallbackReceiver
{
    public int id;
    public int room;
    public bool isRunning;
    public bool isPaused;
    public float time;
    public string timer;
    public string started_at;
    public int heart_bpm;
    public float p_sub;
    public float p_suit;
    public float t_sub;
    public float v_fan;
    public float p_o2;
    public float rate_o2;
    public float batteryPercent;
    public int cap_battery;
    public int battery_out;
    public float p_h2o_g;
    public float p_h2o_l;
    public int p_sop;
    public float rate_sop;
    public string t_battery;
    public int t_oxygenPrimary;
    public float t_oxygenSec;
    public int ox_primary;
    public int ox_secondary;
    public string t_oxygen;
    public float cap_water;
    public string t_water;
    public System.DateTime CreatedAt;
    public System.DateTime UpdatedAt;

    [SerializeField] private string createdAt;
    [SerializeField] private string updatedAt;

    public void OnBeforeSerialize()
    {
        createdAt = CreatedAt.ToString("o");
        updatedAt = UpdatedAt.ToString("o");
    }

    public void OnAfterDeserialize()
    {
        System.DateTime.TryParse(createdAt, out CreatedAt);
        System.DateTime.TryParse(updatedAt, out UpdatedAt);
    }
}
