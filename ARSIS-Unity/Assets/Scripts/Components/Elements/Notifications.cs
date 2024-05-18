using ARSIS.EventManager;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using System.Linq;
using UnityEngine.InputSystem;

public class Notifications : MonoBehaviour, IRenderable
{
    [SerializeField] TextMeshProUGUI connectionText;
    [SerializeField] TextMeshProUGUI missionTimer;
    [SerializeField] TextMeshProUGUI evaText;
    private static string key = "eva";
    private EVA timer = null;
    private bool changed = true;

    void IRenderable.Render(List<BaseArsisEvent> data)
    {
        if (data.LastOrDefault() is EVA recent)
            timer = recent;
        changed = true;
    }

    void Update()
    {
        EventManager eventManager = EventManager.Instance;
        string status = eventManager.Client.GetStatus();
        connectionText.text = status;
        int eva = eventManager.Eva;
        evaText.text = "EVA" + eva;
        if (!changed || timer == null) return;
        string timerText = timer.data.started ? "In Progress" : "Not Started";
        if (timer.data.paused) timerText = "Paused";
        if (timer.data.completed) timerText = "Completed";
        float h = Mathf.Floor(timer.data.total_time / 3600);
        float m = Mathf.Floor(timer.data.total_time % 3600 / 60);
        float s = Mathf.Floor(timer.data.total_time % 3600 % 60);
        missionTimer.text = string.Format("{3}\n{0:00}:{1:00}:{2:00}", h, m, s, timerText);
        changed = false;
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
}
