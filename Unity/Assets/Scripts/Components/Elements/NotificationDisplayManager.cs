using ARSIS.EventManager;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using System.Linq;
using MixedReality.Toolkit.UX;
using UnityEngine.Rendering.VirtualTexturing;
using TMPro;
using UnityEngine.Playables;


/// <summary>
/// Manages the popup notification display. Shows one notification at a time,
/// auto-dismissing after a set duration. The user can also manually dismiss.
/// Hides itself when there are no pending notifications.
/// </summary>
public class NotificationDisplayManager : MonoBehaviour, IRenderable
{
    public List<BaseArsisEvent> data = new();

    [Header("Prefab & Container")]
    [SerializeField] private GameObject mainNotifPrefab;
    [SerializeField] private GameObject mainParentObject;

    [Header("Timing")]
    [SerializeField] private float autoDismissDuration = 5f;

    // Index of the next notification we haven't shown yet
    private int _pendingIndex = 0;

    // The currently displayed popup instance
    private GameObject _currentPopup;

    // How long the current popup has been visible
    private float _displayTimer = 0f;

    // Whether we're actively showing a popup
    private bool _isShowing = false;

    public void Render(List<BaseArsisEvent> data)
    {
        this.data = data;
        // If new notifications arrived beyond what we've processed, start showing
        TryShowNext();
    }

    private void Start()
    {
        EventDatastore.Instance.AddHandler("notification", this);
        HidePopup();
    }

    private void Update()
    {
        if (!_isShowing) return;

        _displayTimer += Time.deltaTime;
        if (_displayTimer >= autoDismissDuration)
            Dismiss();
    }

    /// <summary>
    /// Called by the dismiss button on the popup prefab.
    /// </summary>
    public void DismissButton()
    {
        Dismiss();
    }

    private void Dismiss()
    {
        _pendingIndex++;
        HidePopup();
        TryShowNext();
    }

    private void TryShowNext()
    {
        // Catch up: if there are multiple pending, skip to the latest
        // so the user sees the most recent unread notification next.
        if (_pendingIndex >= data.Count)
        {
            HidePopup();
            return;
        }

        ShowNotification(data[_pendingIndex]);
    }

    private void ShowNotification(BaseArsisEvent baseEvent)
    {
        if (baseEvent is not ARSIS.EventManager.Notifications notification) return;

        // Clear any existing popup
        HidePopup();

        _currentPopup = Instantiate(mainNotifPrefab, mainParentObject.transform);
        _displayTimer = 0f;
        _isShowing = true;

        // Content text
        Transform contentTransform = _currentPopup.transform.Find("MainNotifBackground/Content");
        if (contentTransform != null)
        {
            TextMeshProUGUI tmp = contentTransform.GetComponent<TextMeshProUGUI>();
            if (tmp != null)
                tmp.text = notification.data.content;
            else
                Debug.LogError("NotificationDisplayManager: TextMeshPro not found on Content.");
        }
        else
        {
            Debug.LogError("NotificationDisplayManager: Content transform not found in prefab.");
        }

        // Color band
        Transform colorBandTransform = _currentPopup.transform.Find("MainNotifBackground/ColorBand");
        if (colorBandTransform != null)
        {
            Image colorBandImage = colorBandTransform.GetComponent<Image>();
            NotificationHelper.ApplySeverityColor(colorBandImage, notification.data.severity);
        }
        else
        {
            Debug.LogError("NotificationDisplayManager: ColorBand transform not found in prefab.");
        }

        // Wire up the dismiss button if present
        Transform dismissTransform = _currentPopup.transform.Find("MainNotifBackground/DismissButton");
        if (dismissTransform != null)
        {
            UnityEngine.UI.Button dismissBtn = dismissTransform.GetComponent<UnityEngine.UI.Button>();
            if (dismissBtn != null)
                dismissBtn.onClick.AddListener(DismissButton);
        }
        // Not logging an error here — dismiss button is optional in the prefab.
    }

    private void HidePopup()
    {
        if (_currentPopup != null)
        {
            Destroy(_currentPopup);
            _currentPopup = null;
        }

        _isShowing = false;
        _displayTimer = 0f;
    }

    private void OnDestroy()
    {
        EventDatastore.Instance.RemoveHandler("notification", this);
    }
}


// Old code from NotificationWindowManager for reference:

// public class NotificationDisplayManager : MonoBehaviour, IRenderable
// {
//     private Boolean changed = true;
//     public List<BaseArsisEvent> data = new();
//     [SerializeField]
//     public GameObject MainNotifObj;
//     public GameObject mainParentObject;
//     private int lastNotification = 0;



//     private float cooldownTimer = 0f;
//     private float cooldownDuration = 4f; // Cooldown duration in seconds

//     public void Render(List<BaseArsisEvent> data)
//     {
//         this.data = data;
//         changed = true;
//     }
//     void Start()
//     {
//         EventDatastore eventDatastore = EventDatastore.Instance;
//         eventDatastore.AddHandler("notification", this);
//     }

//     void Update()
//     {
//         SetPopUp();
//         /* if (cooldownTimer >= cooldownDuration)
//          {
//              cooldownTimer = 0f;

//              SetPopUp();
//              //SetMenu();
//          }*/
//     }

//     void SetPopUp()
//     {
//         if (data.Count == 0) return;
//         if(lastNotification >= data.Count)
//         {
            
//             foreach (Transform child in mainParentObject.transform)
//             {
//                 Destroy(child.gameObject);
//             }
//             return;
//         }
//         BaseArsisEvent baseArsisEvent = data[data.Count - 1];

//         if (baseArsisEvent is ARSIS.EventManager.Notifications notification)
//         {
//            // Debug.Log(notification.data.content + "\n" + notification.data.severity);

//             foreach(Transform child in mainParentObject.transform)
//             {
//                 Destroy(child.gameObject);
//             }

//             //instantiating notification object in the scene
//             GameObject mainNotifObj = Instantiate(MainNotifObj, mainParentObject.transform);

//             //updating content text below

//             Transform contentTransform = mainNotifObj.transform.Find("MainNotifBackground/Content");

//             if (contentTransform != null)
//             {
//                 TextMeshProUGUI contentTextMeshPro = contentTransform.GetComponent<TextMeshProUGUI>();

//                 if (contentTextMeshPro != null)
//                 {
//                     contentTextMeshPro.text = notification.data.content;
//                 }
//                 else
//                 {
//                     Debug.LogError("TextMeshPro component not found in Content object.");
//                 }
//             }
//             else
//             {
//                 Debug.LogError("Content TextMeshPro object not found in MainNotifObj prefab.");
//             }

//             //update timestamp
//           /*  Transform timeTransform = mainNotifObj.transform.Find("MainNotifBackground/TimeStamp");

//             if (timeTransform != null)
//             {
//                 TextMeshProUGUI timeTMP = timeTransform.GetComponent<TextMeshProUGUI>();

//                 if (timeTMP != null)
//                 {
//                     timeTMP.text = notification.data.time.ToString();
//                 }
//                 else
//                 {
//                     Debug.LogError("TextMeshPro component not found in TimeStamp object.");
//                 }
//             }
//             else
//             {
//                 Debug.LogError("Content TextMeshPro object not found in MainNotifObj prefab.");
//             }
//           */

//             // Update ColorBand Image color
//             Transform colorBandTransform = mainNotifObj.transform.Find("MainNotifBackground/ColorBand");
//             if (colorBandTransform != null)
//             {
//                 Image colorBandImage = colorBandTransform.GetComponent<Image>();
//                 if (colorBandImage != null)
//                 {
//                     // Set color based on severity rating
//                     switch (notification.data.severity)
//                     {
//                         case 0:
//                             colorBandImage.color = Color.red;
//                             break;
//                         case 1:
//                             colorBandImage.color = Color.yellow;
//                             break;
//                         case 2:
//                             colorBandImage.color = new Color(0.5f, 0f, 0.5f); // Purple RGB value
//                             break;
//                         default:
//                             colorBandImage.color = Color.white;
//                             break;
//                     }
//                 }
//                 else
//                 {
//                     Debug.LogError("Image component not found in ColorBand object.");
//                 }
//             }
//             else
//             {
//                 Debug.LogError("ColorBand object not found in MainNotifObj prefab.");
//             }

//             // DestroyAfterDelay(mainNotifObj, 4f);

//             cooldownTimer += Time.deltaTime;
//             if (cooldownTimer >= cooldownDuration)
//             {
//                 cooldownTimer = 0f;
//                 lastNotification += 1;
//             }
//         }

//     }

//   /*  void DestroyAfterDelay(GameObject obj, float delay)
//     {
//         Destroy(obj, delay);
//     }*/

// }
