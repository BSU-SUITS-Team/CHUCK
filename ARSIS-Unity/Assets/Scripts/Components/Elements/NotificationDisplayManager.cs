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

public class NotificationDisplayManager : MonoBehaviour, IRenderable
{
    private Boolean changed = true;
    public List<BaseArsisEvent> data = new();
    [SerializeField]
    public GameObject MainNotifObj;
    public GameObject mainParentObject;
    


    private float cooldownTimer = 0f;
    private float cooldownDuration = 10f; // Cooldown duration in seconds

    public void Render(List<BaseArsisEvent> data)
    {
        this.data = data;
        changed = true;
    }
    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler("notification", this);
    }

    void Update()
    {
        cooldownTimer += Time.deltaTime;

        if (cooldownTimer >= cooldownDuration)
        {
            cooldownTimer = 0f;

            SetPopUp();
            //SetMenu();
        }
    }

    void SetPopUp()
    {
        foreach (BaseArsisEvent baseArsisEvent in data)
        {
            if (baseArsisEvent is ARSIS.EventManager.Notifications notification)
            {
                Debug.Log(notification.data.content + "\n" + notification.data.severity);

                //instantiating notification object in the scene
                GameObject mainNotifObj = Instantiate(MainNotifObj, mainParentObject.transform);

                //updating content text below

                Transform contentTransform = mainNotifObj.transform.Find("MainNotifBackground/Content");

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

                //update timestamp
                Transform timeTransform = mainNotifObj.transform.Find("MainNotifBackground/TimeStamp");

                if (timeTransform != null)
                {
                    TextMeshProUGUI timeTMP = timeTransform.GetComponent<TextMeshProUGUI>();

                    if (timeTMP != null)
                    {
                        timeTMP.text = notification.data.time.ToString();
                    }
                    else
                    {
                        Debug.LogError("TextMeshPro component not found in TimeStamp object.");
                    }
                }
                else
                {
                    Debug.LogError("Content TextMeshPro object not found in MainNotifObj prefab.");
                }

                // Update ColorBand Image color
                Transform colorBandTransform = mainNotifObj.transform.Find("MainNotifBackground/ColorBand");
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

                DestroyAfterDelay(mainNotifObj, 4f);
            }
        }
    }

    void DestroyAfterDelay(GameObject obj, float delay)
    {
        Destroy(obj, delay);
    }

}
