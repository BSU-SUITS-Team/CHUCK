using ARSIS.EventManager;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Linq;
using MixedReality.Toolkit.UX;
using UnityEngine.Rendering.VirtualTexturing;

public class NotificationDisplayManager : MonoBehaviour, IRenderable
{
    private Boolean changed = true;
    private List<BaseArsisEvent> data = new();
    [SerializeField]
    public GameObject MainNotifObj;
    public GameObject mainParentObject;

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
        Debug.Log(data);
        foreach (BaseArsisEvent baseArsisEvent in data)
        {
            if (baseArsisEvent is ARSIS.EventManager.Notifications notification)
            {
                Debug.Log(notification.data.content);

                GameObject mainNotifObj = Instantiate(MainNotifObj, mainParentObject.transform);
            }
        }
    }
}
