using UnityEngine;
using ARSIS.EventManager;
using System;

/// <summary>
/// Watches TSSConnectionManager for error latch changes and pushes
/// Notifications events into EventDatastore. This is the bridge between
/// the new TSS polling system and the old ARSIS notification system.
/// 
/// Fires once when an error is latched, and once when it is reset.
/// </summary>
public class TSSNotificationWatcher : MonoBehaviour
{
    // Track previous latch states so we only fire on transitions
    private bool _prevFanLatched;
    private bool _prevOxyLatched;
    private bool _prevPowerLatched;
    private bool _prevScrubberLatched;

    private TSSConnectionManager Manager => TSSConnectionManager.Instance;

    private void OnEnable()
    {
        if (Manager == null)
        {
            Debug.LogError("TSSNotificationWatcher: no TSSConnectionManager found.");
            return;
        }

        Manager.ErrorLatchUpdated += HandleErrorLatchUpdated;

        // Sync initial state without firing notifications —
        // we don't want to re-notify for errors that were already latched
        // before this component enabled (e.g. scene reload).
        SyncInitialState();
    }

    private void OnDisable()
    {
        if (Manager != null)
            Manager.ErrorLatchUpdated -= HandleErrorLatchUpdated;
    }

    private void SyncInitialState()
    {
        if (Manager == null) return;
        _prevFanLatched     = Manager.FanErrorLatch.latched;
        _prevOxyLatched     = Manager.OxyErrorLatch.latched;
        _prevPowerLatched   = Manager.PowerErrorLatch.latched;
        _prevScrubberLatched = Manager.ScrubberErrorLatch.latched;
    }

    private void HandleErrorLatchUpdated()
    {
        if (Manager == null) return;

        CheckLatch("Fan",     Manager.FanErrorLatch.latched,     ref _prevFanLatched,
                   "Fan error detected.",   "Fan error cleared.", "Off Nominal Primary Fan");

        CheckLatch("Oxy",     Manager.OxyErrorLatch.latched,     ref _prevOxyLatched,
                   "Oxygen error detected.", "Oxygen error cleared.", "Off Nominal Suit Oxygen Pressure");

        CheckLatch("Power",   Manager.PowerErrorLatch.latched,   ref _prevPowerLatched,
                   "Power error detected.", "Power error cleared.", "Off Nominal Battery Level");

        CheckLatch("Scrubber", Manager.ScrubberErrorLatch.latched, ref _prevScrubberLatched,
                   "Scrubber error detected.", "Scrubber error cleared.", "Off Nomincal CO2 Scrubber");
    }

    private void CheckLatch(string label, bool currentLatched, ref bool previousLatched,
                            string tripMessage, string clearMessage, string fixProc)
    {
        if (currentLatched && !previousLatched)
        {
            // Error just latched — notify
            PushNotification(label, tripMessage, severity: 0, fixProc);
        }
        else if (!currentLatched && previousLatched)
        {
            // Error was reset — confirm
            PushNotification(label + "_cleared", clearMessage, severity: 1, fixProc);
        }

        previousLatched = currentLatched;
    }

    private void PushNotification(string label, string content, int severity, string fixProcedure)
    {
        var notification = new ARSIS.EventManager.Notifications
        {
            type  = "notification",
            label = label,
            time  = DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
            data  = new NotificationsData
            {
                content  = content,
                severity = severity,
                procedure = fixProcedure,
                time     = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds()
            }
        };

        EventDatastore.Instance.Append("notification", notification);
    }
}
