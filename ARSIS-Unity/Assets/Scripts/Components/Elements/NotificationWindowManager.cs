using ARSIS.EventManager;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class NotificationWindowManager : MonoBehaviour
{
    private NotificationDisplayManager _notificationDisplayManager;
    public GameObject MiniNotifObj;
    public GameObject miniParentObject;
    void Start()
    {
        _notificationDisplayManager = FindObjectOfType<NotificationDisplayManager>();
    }

    void Update()
    {
        SetMenu();
    }

    void SetMenu()
    {
        foreach (Transform child in miniParentObject.transform)
        {
            Destroy(child.gameObject);
        }
        foreach (BaseArsisEvent baseArsisEvent in _notificationDisplayManager.data)
        {
            if (baseArsisEvent is ARSIS.EventManager.Notifications notification)
            {
                //instantiating notification object in the scene
                GameObject miniNotifObj = Instantiate(MiniNotifObj, miniParentObject.transform);

                //updating content text below

                Transform contentTransform = miniNotifObj.transform.Find("MainNotifBackground/Content");
                if (contentTransform != null)
                {
                    TextMeshProUGUI contentTextMeshPro = contentTransform.GetComponent<TextMeshProUGUI>();

                    if (contentTextMeshPro != null)
                    {
                        contentTextMeshPro.text = notification.data.content;
                    }
                    else
                    {
                        Debug.LogError("TextMeshPro component not found in Content object.");
                    }
                }
                else
                {
                    Debug.LogError("Content TextMeshPro object not found in MainNotifObj prefab.");
                }
                //updating timestamp below

            /*    Transform timeTransform = miniNotifObj.transform.Find("MainNotifBackground/TimeStamp");
                if (timeTransform != null)
                {
                    TextMeshProUGUI timeTMP = timeTransform.GetComponent<TextMeshProUGUI>();

                    if (timeTMP != null)
                    {
                        timeTMP.text = notification.data.time.ToString();
                    }
                    else
                    {
                        Debug.LogError("TextMeshPro component not found in Content object.");
                    }
                }
                else
                {
                    Debug.LogError("Content TextMeshPro object not found in MainNotifObj prefab.");
                }*/

                //updating color below

                Transform colorBandTransform = miniNotifObj.transform.Find("MainNotifBackground/ColorBand");
                if (colorBandTransform != null)
                {
                    Image colorBandImage = colorBandTransform.GetComponent<Image>();
                    if (colorBandImage != null)
                    {
                        // Set color based on severity rating
                        switch (notification.data.severity)
                        {
                            case 0:
                                colorBandImage.color = Color.red;
                                break;
                            case 1:
                                colorBandImage.color = Color.yellow;
                                break;
                            case 2:
                                colorBandImage.color = new Color(0.5f, 0f, 0.5f); // Purple RGB value
                                break;
                            default:
                                colorBandImage.color = Color.white;
                                break;
                        }
                    }
                    else
                    {
                        Debug.LogError("Image component not found in ColorBand object.");
                    }
                }
                else
                {
                    Debug.LogError("ColorBand object not found in MainNotifObj prefab.");
                }
            }
        }
    }

}
