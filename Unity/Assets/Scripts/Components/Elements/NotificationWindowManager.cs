using ARSIS.EventManager;
using MixedReality.Toolkit.UX;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Manages the notification history window. Displays all notifications
/// as a scrollable list, rebuilt only when new data arrives (not every frame).
/// Includes a Reset All Errors button and live time-since-detection display.
/// </summary>
public class NotificationWindowManager : MonoBehaviour, IRenderable
{
    [Header("Prefab & Container")]
    [SerializeField] private GameObject miniNotifPrefab;
    [SerializeField] private GameObject miniParentObject;

    [Header("Reset All Button")]
    [SerializeField] private Button resetAllButton;

    [Header("Clear History Button")]
    [SerializeField] private PressableButton clearHistoryButton;

    // Local copy of notification data
    private List<BaseArsisEvent> _data = new();

    // Track instantiated rows so we can update their timers each frame
    private readonly List<NotificationRow> _rows = new();

    private TSSConnectionManager Manager => TSSConnectionManager.Instance;

    private void Start()
    {
        EventDatastore.Instance.AddHandler("notification", this);

        if (resetAllButton != null)
            resetAllButton.onClick.AddListener(OnResetAll);
        else
            Debug.LogWarning("NotificationWindowManager: no Reset All button assigned.");

        clearHistoryButton?.OnClicked.AddListener(OnClearHistory);
    }

    private void OnDestroy()
    {
        EventDatastore.Instance.RemoveHandler("notification", this);

        if (resetAllButton != null)
            resetAllButton.onClick.RemoveListener(OnResetAll);

        clearHistoryButton?.OnClicked.RemoveListener(OnClearHistory);
    }

    // Called by EventDatastore when notification data changes
    public void Render(List<BaseArsisEvent> data)
    {
        _data = data;
        RebuildList();
    }

    private void Update()
    {
        // Update elapsed time labels on each row every frame
        foreach (NotificationRow row in _rows)
            row.UpdateTimer();
    }

    private void RebuildList()
    {
        // Clear existing rows
        foreach (Transform child in miniParentObject.transform)
            Destroy(child.gameObject);

        _rows.Clear();

        foreach (BaseArsisEvent baseEvent in _data)
        {
            if (baseEvent is not ARSIS.EventManager.Notifications notification) continue;

            GameObject obj = Instantiate(miniNotifPrefab, miniParentObject.transform);

            // Content
            Transform contentTransform = obj.transform.Find("MainNotifBackground/Content");
            if (contentTransform != null)
            {
                TextMeshProUGUI tmp = contentTransform.GetComponent<TextMeshProUGUI>();
                if (tmp != null)
                    tmp.text = notification.data.content;
                else
                    Debug.LogError("NotificationWindowManager: TextMeshPro not found on Content.");
            }
            else
            {
                Debug.LogError("NotificationWindowManager: Content transform not found in prefab.");
            }

            // Color band
            Transform colorBandTransform = obj.transform.Find("MainNotifBackground/ColorBand");
            if (colorBandTransform != null)
            {
                Image colorBandImage = colorBandTransform.GetComponent<Image>();
                NotificationHelper.ApplySeverityColor(colorBandImage, notification.data.severity);
            }
            else
            {
                Debug.LogError("NotificationWindowManager: ColorBand transform not found in prefab.");
            }

            // Time since detection label (optional in prefab)
            TMP_Text timerLabel = null;
            Transform timerTransform = obj.transform.Find("MainNotifBackground/TimeElapsed");
            if (timerTransform != null)
                timerLabel = timerTransform.GetComponent<TMP_Text>();

            // Register row so Update() can tick its timer
            _rows.Add(new NotificationRow(notification, timerLabel));
        }
    }

    public void OnClearHistory()
    {
        EventDatastore.Instance.ClearKey("notification");
    }

    private void OnResetAll()
    {
        if (Manager == null)
        {
            Debug.LogError("NotificationWindowManager: no TSSConnectionManager found.");
            return;
        }

        Manager.ResetAllErrors();
        EventDatastore.Instance.ClearKey("notification");
    }

    // -------------------------------------------------------------------------
    // Inner class — tracks one row's detection time and updates its label
    // -------------------------------------------------------------------------
    private class NotificationRow
    {
        private readonly ARSIS.EventManager.Notifications _notification;
        private readonly TMP_Text _timerLabel;
        private readonly float _detectedAtUnscaled;

        public NotificationRow(ARSIS.EventManager.Notifications notification, TMP_Text timerLabel)
        {
            _notification = notification;
            _timerLabel   = timerLabel;

            // notification.data.time is a Unix timestamp (seconds).
            // We convert to local unscaled time so the timer is consistent
            // with TSSConnectionManager's ErrorLatch.SecondsSinceTrip.
            long nowUnix = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
            float secondsAgo = Mathf.Max(0f, nowUnix - _notification.data.time);
            _detectedAtUnscaled = Time.unscaledTime - secondsAgo;
        }

        public void UpdateTimer()
        {
            if (_timerLabel == null) return;

            float elapsed = Time.unscaledTime - _detectedAtUnscaled;
            _timerLabel.text = elapsed >= 0f
                ? FormatElapsed(elapsed)
                : "-";
        }

        private static string FormatElapsed(float seconds)
        {
            if (seconds < 60f)
                return $"{seconds:F0}s ago";

            int m = (int)(seconds / 60f);
            int s = (int)(seconds % 60f);
            return $"{m}m {s:00}s ago";
        }
    }
}



// Old code from NotificationWindowManager for reference:

// public class NotificationWindowManager : MonoBehaviour
// {
//     private NotificationDisplayManager _notificationDisplayManager;
//     public GameObject MiniNotifObj;
//     public GameObject miniParentObject;
//     void Start()
//     {
//         _notificationDisplayManager = FindObjectOfType<NotificationDisplayManager>();
//     }

//     void Update()
//     {
//         SetMenu();
//     }

//     void SetMenu()
//     {
//         foreach (Transform child in miniParentObject.transform)
//         {
//             Destroy(child.gameObject);
//         }
//         foreach (BaseArsisEvent baseArsisEvent in _notificationDisplayManager.data)
//         {
//             if (baseArsisEvent is ARSIS.EventManager.Notifications notification)
//             {
//                 //instantiating notification object in the scene
//                 GameObject miniNotifObj = Instantiate(MiniNotifObj, miniParentObject.transform);

//                 //updating content text below

//                 Transform contentTransform = miniNotifObj.transform.Find("MainNotifBackground/Content");
//                 if (contentTransform != null)
//                 {
//                     TextMeshProUGUI contentTextMeshPro = contentTransform.GetComponent<TextMeshProUGUI>();

//                     if (contentTextMeshPro != null)
//                     {
//                         contentTextMeshPro.text = notification.data.content;
//                     }
//                     else
//                     {
//                         Debug.LogError("TextMeshPro component not found in Content object.");
//                     }
//                 }
//                 else
//                 {
//                     Debug.LogError("Content TextMeshPro object not found in MainNotifObj prefab.");
//                 }
//                 //updating timestamp below

//             /*    Transform timeTransform = miniNotifObj.transform.Find("MainNotifBackground/TimeStamp");
//                 if (timeTransform != null)
//                 {
//                     TextMeshProUGUI timeTMP = timeTransform.GetComponent<TextMeshProUGUI>();

//                     if (timeTMP != null)
//                     {
//                         timeTMP.text = notification.data.time.ToString();
//                     }
//                     else
//                     {
//                         Debug.LogError("TextMeshPro component not found in Content object.");
//                     }
//                 }
//                 else
//                 {
//                     Debug.LogError("Content TextMeshPro object not found in MainNotifObj prefab.");
//                 }*/

//                 //updating color below

//                 Transform colorBandTransform = miniNotifObj.transform.Find("MainNotifBackground/ColorBand");
//                 if (colorBandTransform != null)
//                 {
//                     Image colorBandImage = colorBandTransform.GetComponent<Image>();
//                     if (colorBandImage != null)
//                     {
//                         // Set color based on severity rating
//                         switch (notification.data.severity)
//                         {
//                             case 0:
//                                 colorBandImage.color = Color.red;
//                                 break;
//                             case 1:
//                                 colorBandImage.color = Color.yellow;
//                                 break;
//                             case 2:
//                                 colorBandImage.color = new Color(0.5f, 0f, 0.5f); // Purple RGB value
//                                 break;
//                             default:
//                                 colorBandImage.color = Color.white;
//                                 break;
//                         }
//                     }
//                     else
//                     {
//                         Debug.LogError("Image component not found in ColorBand object.");
//                     }
//                 }
//                 else
//                 {
//                     Debug.LogError("ColorBand object not found in MainNotifObj prefab.");
//                 }
//             }
//         }
//     }

// }
