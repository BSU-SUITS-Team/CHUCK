using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class TSSLatchedErrorView : MonoBehaviour
{
    [Header("Fan Error")]
    [SerializeField] private TMP_Text fanLiveText;
    [SerializeField] private TMP_Text fanLatchedText;
    [SerializeField] private TMP_Text fanSecondsText;

    [Header("Oxy Error")]
    [SerializeField] private TMP_Text oxyLiveText;
    [SerializeField] private TMP_Text oxyLatchedText;
    [SerializeField] private TMP_Text oxySecondsText;

    [Header("Power Error")]
    [SerializeField] private TMP_Text powerLiveText;
    [SerializeField] private TMP_Text powerLatchedText;
    [SerializeField] private TMP_Text powerSecondsText;

    [Header("Scrubber Error")]
    [SerializeField] private TMP_Text scrubberLiveText;
    [SerializeField] private TMP_Text scrubberLatchedText;
    [SerializeField] private TMP_Text scrubberSecondsText;

    private TSSConnectionManager Manager => TSSConnectionManager.Instance;

    private void OnEnable()
    {
        if (Manager == null)
        {
            Debug.LogError("TSSLatchedErrorView: no TSSConnectionManager found.");
            return;
        }

        Manager.EvaUpdated += HandleEvaUpdated;
        Manager.ErrorLatchUpdated += Refresh;

        Refresh();
    }

    private void OnDisable()
    {
        if (Manager == null) return;

        Manager.EvaUpdated -= HandleEvaUpdated;
        Manager.ErrorLatchUpdated -= Refresh;
    }

    private void Update()
    {
        // Keep elapsed seconds updating while latched.
        RefreshSecondsOnly();
    }

    private void HandleEvaUpdated(EvaRoot _)
    {
        Refresh();
    }

    private void Refresh()
    {
        if (Manager == null) return;

        SetBool(fanLiveText, Manager.FanErrorLatch.live);
        SetBool(fanLatchedText, Manager.FanErrorLatch.latched);

        SetBool(oxyLiveText, Manager.OxyErrorLatch.live);
        SetBool(oxyLatchedText, Manager.OxyErrorLatch.latched);

        SetBool(powerLiveText, Manager.PowerErrorLatch.live);
        SetBool(powerLatchedText, Manager.PowerErrorLatch.latched);

        SetBool(scrubberLiveText, Manager.ScrubberErrorLatch.live);
        SetBool(scrubberLatchedText, Manager.ScrubberErrorLatch.latched);

        RefreshSecondsOnly();
    }

    private void RefreshSecondsOnly()
    {
        if (Manager == null) return;

        SetSeconds(fanSecondsText, Manager.FanErrorLatch);
        SetSeconds(oxySecondsText, Manager.OxyErrorLatch);
        SetSeconds(powerSecondsText, Manager.PowerErrorLatch);
        SetSeconds(scrubberSecondsText, Manager.ScrubberErrorLatch);
    }

    private void SetBool(TMP_Text target, bool value)
    {
        if (target == null) return;
        target.text = value ? "ON" : "OFF";
    }

    private void SetSeconds(TMP_Text target, ErrorLatch latch)
    {
        if (target == null) return;
        target.text = latch.latched ? latch.SecondsSinceTrip.ToString("F1") : "-";
    }

    // Button hooks
    public void ResetFanError()
    {
        Manager?.ResetFanError();
    }

    public void ResetOxyError()
    {
        Manager?.ResetOxyError();
    }

    public void ResetPowerError()
    {
        Manager?.ResetPowerError();
    }

    // public void ResetScrubberError()
    // {
    //     Manager?.ResetScrubberError();
    // }

    public void ResetAllErrors()
    {
        Manager?.ResetAllErrors();
    }
}
