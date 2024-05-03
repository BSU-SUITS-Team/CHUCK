using ARSIS.EventManager;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Linq;

public class Biometrics : MonoBehaviour, IRenderable
{
    // Suit Resources
    [SerializeField] vertGauge2Drain battery;
    [SerializeField] vertGauge2Drain oxyPriSto;
    [SerializeField] vertGauge2Drain oxySecSto;
    [SerializeField] vertGauge2Drain oxyPriPre;
    [SerializeField] vertGauge2Drain oxySecPre;
    [SerializeField] vertGauge2Drain oxyTime;
    [SerializeField] vertGauge2Drain coolantSto;

    // Suit Atmosphere
    [SerializeField] archGauge heartRate;
    [SerializeField] archGauge oxyConsump;
    [SerializeField] archGauge co2Prod;
    [SerializeField] archGauge suitPreO2;
    [SerializeField] vertGaugeBuild3 suitPreCO2;
    [SerializeField] vertGaugeBuild3 suitPreOther;
    [SerializeField] archGauge suitPreTotal;
    [SerializeField] vertGaugeBuild3 helmPreCO2;

    // Fan
    [SerializeField] vertGauge2Drain fanPri;
    [SerializeField] vertGauge2Drain fanSec;

    // Scrubber
    [SerializeField] vertGaugeSplit1 scrubberA;
    [SerializeField] vertGaugeSplit1 scrubberB;

    // Temperature
    [SerializeField] archGauge temperature;
    [SerializeField] archGauge coolantLiquid;
    [SerializeField] vertGaugeBuild3 coolantGas;


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
        battery.currentValue = evaData.batt_time_left;
        oxyPriSto.currentValue = evaData.oxy_pri_storage;
        oxySecSto.currentValue = evaData.oxy_sec_storage;
        oxyPriPre.currentValue = evaData.oxy_pri_pressure;
        oxySecPre.currentValue = evaData.oxy_sec_pressure;
        oxyTime.currentValue = evaData.oxy_time_left;
        coolantSto.currentValue = evaData.coolant_ml;

        heartRate.biosValue = evaData.heart_rate;
        oxyConsump.biosValue = evaData.oxy_consumption;
        co2Prod.biosValue = evaData.co2_production;
        suitPreO2.biosValue = evaData.suit_pressure_oxy;
        suitPreCO2.currentValue = evaData.suit_pressure_co2;
        suitPreOther.currentValue = evaData.suit_pressure_other;
        suitPreTotal.biosValue = evaData.suit_pressure_total;
        helmPreCO2.currentValue = evaData.helmet_pressure_co2;

        fanPri.currentValue = evaData.fan_pri_rpm;
        fanSec.currentValue = evaData.fan_sec_rpm;

        scrubberA.currentValue = evaData.scrubber_a_co2_storage;
        scrubberB.currentValue = evaData.scrubber_b_co2_storage;

        temperature.biosValue = evaData.temperature;
        coolantLiquid.biosValue = evaData.coolant_liquid_pressure;
        coolantGas.currentValue = evaData.coolant_gas_pressure;
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
