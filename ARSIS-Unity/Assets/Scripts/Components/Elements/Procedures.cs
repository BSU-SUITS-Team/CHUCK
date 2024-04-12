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

    void IRenderable.Render(List<BaseArsisEvent> list)
    {
        List<GameObject> entries = new();
        procedures = list;
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
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(key, this);
    }

    private void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(key, this);
    }
}
