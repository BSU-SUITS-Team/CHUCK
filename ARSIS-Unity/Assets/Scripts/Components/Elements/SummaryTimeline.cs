using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using UnityEngine;

public class SummaryTimeline : MonoBehaviour, IRenderable
{
    private List<BaseArsisEvent> data;
    private bool changed = true;
    private string key = "eva";

    void IRenderable.Render(List<BaseArsisEvent> data)
    {
        this.data = data;
        changed = true;
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(key, this);
    }

    void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(key, this);
    }

    void Update()
    {
        if (!changed) return;
        changed = true;
    }
}
