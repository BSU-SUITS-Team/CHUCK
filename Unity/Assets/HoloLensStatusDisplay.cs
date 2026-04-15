using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using System;

#if WINDOWS_UWP
using Windows.Devices.Power;
#endif

public class HoloLensStatusDisplay : MonoBehaviour
{
    [Header("Text References")]
    [SerializeField] private TMP_Text batteryText;
    [SerializeField] private TMP_Text timeText;

    [Header("Update Rates")]
    [SerializeField] private float batteryUpdateIntervalSeconds = 5f;
    [SerializeField] private float timeUpdateIntervalSeconds = 1f;

    private float batteryTimer;
    private float timeTimer;

    private void Start()
    {
        UpdateBatteryText();
        UpdateTimeText();
    }

    private void Update()
    {
        batteryTimer += Time.deltaTime;
        timeTimer += Time.deltaTime;

        if (batteryTimer >= batteryUpdateIntervalSeconds)
        {
            batteryTimer = 0f;
            UpdateBatteryText();
        }

        if (timeTimer >= timeUpdateIntervalSeconds)
        {
            timeTimer = 0f;
            UpdateTimeText();
        }
    }

    private void UpdateTimeText()
    {
        if (timeText == null)
            return;

        timeText.text = DateTime.Now.ToString("HH:mm");
    }

    private void UpdateBatteryText()
    {
        if (batteryText == null)
            return;

#if WINDOWS_UWP && !UNITY_EDITOR
        var battery = Battery.AggregateBattery;
        var report = battery.GetReport();

        if (report != null &&
            report.RemainingCapacityInMilliwattHours.HasValue &&
            report.FullChargeCapacityInMilliwattHours.HasValue &&
            report.FullChargeCapacityInMilliwattHours.Value > 0)
        {
            float remaining = report.RemainingCapacityInMilliwattHours.Value;
            float full = report.FullChargeCapacityInMilliwattHours.Value;
            int percent = Mathf.RoundToInt((remaining / full) * 100f);

            batteryText.text = percent + "%";
        }
        else
        {
            batteryText.text = "--%";
        }
#else
        batteryText.text = "Editor";
#endif
    }
}
