using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

#if WINDOWS_UWP
using Windows.Devices.Power;
#endif

public class HoloLensBatteryTextEvent : MonoBehaviour
{
    [SerializeField] private TMP_Text batteryText;
    [SerializeField] private string prefix = "";

#if WINDOWS_UWP && !UNITY_EDITOR
    private Battery battery;
#endif

    private void Start()
    {
        UpdateBatteryText();
    }

    private void OnEnable()
    {
#if WINDOWS_UWP && !UNITY_EDITOR
        battery = Battery.AggregateBattery;
        if (battery != null)
            battery.ReportUpdated += OnBatteryReportUpdated;
#endif
    }

    private void OnDisable()
    {
#if WINDOWS_UWP && !UNITY_EDITOR
        if (battery != null)
            battery.ReportUpdated -= OnBatteryReportUpdated;
#endif
    }

#if WINDOWS_UWP && !UNITY_EDITOR
    private void OnBatteryReportUpdated(Battery sender, object args)
    {
        UnityEngine.WSA.Application.InvokeOnAppThread(UpdateBatteryText, false);
    }
#endif

    private void UpdateBatteryText()
    {
        if (batteryText == null)
            return;

#if WINDOWS_UWP && !UNITY_EDITOR
        var report = Battery.AggregateBattery.GetReport();

        if (report != null &&
            report.RemainingCapacityInMilliwattHours.HasValue &&
            report.FullChargeCapacityInMilliwattHours.HasValue &&
            report.FullChargeCapacityInMilliwattHours.Value > 0)
        {
            float remaining = report.RemainingCapacityInMilliwattHours.Value;
            float full = report.FullChargeCapacityInMilliwattHours.Value;
            int percent = Mathf.RoundToInt((remaining / full) * 100f);

            batteryText.text = $"{prefix}{percent}%";
        }
        else
        {
            batteryText.text = $"{prefix}--%";
        }
#else
        batteryText.text = $"{prefix}Editor";
#endif
    }
}