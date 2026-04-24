using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class HoloLensTimeText : MonoBehaviour
{
    [SerializeField] private TMP_Text timeText;
    [SerializeField] private float updateIntervalSeconds = 1f;

    private void Start()
    {
        UpdateTimeText();
        InvokeRepeating(nameof(UpdateTimeText), updateIntervalSeconds, updateIntervalSeconds);
    }

    private void OnDestroy()
    {
        CancelInvoke();
    }

    private void UpdateTimeText()
    {
        if (timeText == null)
            return;

        timeText.text = DateTime.Now.ToString("HH:mm");
    }
}
