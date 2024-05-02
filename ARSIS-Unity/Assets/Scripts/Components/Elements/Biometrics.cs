using ARSIS.EventManager;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Linq;

public class Biometrics : MonoBehaviour, IRenderable
{
    [SerializeField] archGauge heartRate;
    [SerializeField] vertGauge2Drain battery;
    [SerializeField] vertGaugeBuild3 scrubberA;
    private Boolean changed = true;
    private List<BaseArsisEvent> data = new();
    private TelemetryEva evaData;
    private string key = "telemetry";

    public void Render(List<BaseArsisEvent> data)
    {
        this.data = data;
        changed = true;
    }

    private void RetrieveEva(int eva)
    {
        Telemetry telemetry = (Telemetry)data.Last();
        evaData = eva switch
        {
            2 => telemetry.data.eva2, // eva2
            _ => telemetry.data.eva1, // default or eva1
        };
    }

    private void UpdateGauges()
    {
        heartRate.biosValue = evaData.heart_rate;
        battery.currentValue = evaData.batt_time_left;
        scrubberA.currentValue = evaData.scrubber_a_co2_storage;
    }

    void Update()
    {
        if (!changed || data == null || data.Count == 0) return;
        if (data.Last() is not Telemetry) return;
        EventManager eventManager = EventManager.Instance;
        RetrieveEva(eventManager.Eva);
        UpdateGauges();
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(key, this);
    }

    void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(key, this);
    }
}
