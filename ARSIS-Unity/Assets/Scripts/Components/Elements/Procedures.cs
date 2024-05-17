using ARSIS.EventManager;
using ARSIS.UI;
using MixedReality.Toolkit.Experimental;
using MixedReality.Toolkit.UX;
using MixedReality.Toolkit.UX.Experimental;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Procedures : MonoBehaviour, IRenderable
{
    [SerializeField] GameObject procedureButton;
    [SerializeField] ScrollArea scrollArea;
    [SerializeField] GameObject procedureDisplay;
    [SerializeField] GameObject summaryTimeline;
    private static string key = "procedure";
    private List<BaseArsisEvent> procedures = new List<BaseArsisEvent>();
    private bool changed = true;

    void IRenderable.Render(List<BaseArsisEvent> list)
    {
        procedures = list;
        changed = true;
    }

    void CreateProcedureDisplay(Procedure procedure)
    {
        GameObject display = Instantiate(procedureDisplay); // procedureDisplay prefab is active = false by default
        ProcedureDisplay view = display.GetComponent<ProcedureDisplay>();
        view.SetProcedure(procedure); // apply the procedure
        display.SetActive(true); // enable after procedure is applied
    }

    public void ShowSummaryTimeline()
    {
        Instantiate(summaryTimeline);
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
                PressableButton pressableButton = button.GetPressableButton();
                pressableButton.OnClicked.AddListener(() => CreateProcedureDisplay(procedure));
                entries.Add(entry);
            }
        }
        scrollArea.SetEntries(entries);
        changed = false;
    }
}
