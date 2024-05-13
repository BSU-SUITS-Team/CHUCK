using System.Collections;
using System.Collections.Generic;
using System.Threading;
using ARSIS.EventManager;
using UnityEngine;
using System.Linq;

public class SummaryTimeline : MonoBehaviour, IRenderable
{
    [SerializeField] RectTransform handle;
    private const float cutoffInSeconds = 3300f;
    private float timerHeight = -250f;
    private EVA time;
    private bool changed = true;
    private string key = "eva";

    void IRenderable.Render(List<BaseArsisEvent> data)
    {
        if (data.Last() is EVA newTime) time = newTime;
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
        float newPos = Mathf.Clamp((time.data.total_time / cutoffInSeconds) * timerHeight, -250, 0);
        handle.anchoredPosition = new Vector2(handle.anchoredPosition.x, newPos);
        changed = true;
    }
}
