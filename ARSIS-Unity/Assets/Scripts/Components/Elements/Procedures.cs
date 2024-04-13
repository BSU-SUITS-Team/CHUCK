using ARSIS.EventManager;
using ARSIS.UI;
using MixedReality.Toolkit.Experimental;
using MixedReality.Toolkit.UX.Experimental;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Procedures : MonoBehaviour, IRenderable
{
    [SerializeField] GameObject procedureButton;
    [SerializeField] ScrollArea scrollArea;
    private static string key = "procedure";
    private List<BaseArsisEvent> procedures = new List<BaseArsisEvent>();
    private bool changed = true;

    void IRenderable.Render(List<BaseArsisEvent> list)
    {
        procedures = list;
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
        List<GameObject> entries = new();
        foreach (BaseArsisEvent baseArsisEvent in procedures)
        {
            if (baseArsisEvent is Procedure procedure)
            {
                GameObject entry = Instantiate(procedureButton);
                Button button = entry.GetComponent<Button>();
                button.SetText(procedure.data.name);
                entries.Add(entry);
            }
        }
        scrollArea.SetEntries(entries);
        changed = false;
    }
}
