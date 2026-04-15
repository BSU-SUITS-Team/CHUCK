using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

#if WINDOWS_UWP
using Windows.Devices.Power;
#endif

public class HoloLensBatteryText : MonoBehaviour
{
    [SerializeField] private TMP_Text batteryText;
    [SerializeField] private float updateIntervalSeconds = 5f;
    [SerializeField] private string prefix = "";

    private void Start()
    {
        UpdateBatteryText();
        InvokeRepeating(nameof(UpdateBatteryText), updateIntervalSeconds, updateIntervalSeconds);
    }

    private void OnDestroy()
    {
        CancelInvoke();
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
