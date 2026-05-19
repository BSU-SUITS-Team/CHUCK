using System;
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
    
    [Tooltip("If this is enabled, this script is only here for reference and should not make changes to the menu")]
    public bool noMenuChanges;

    void IRenderable.Render(List<BaseArsisEvent> list)
    {
        procedures = list;
        changed = true;
    }

    public void CreateProcedureDisplay(Procedure procedure)
    {
        GameObject display = Instantiate(procedureDisplay); // procedureDisplay prefab is active = false by default
        ProcedureDisplay view = display.GetComponent<ProcedureDisplay>();
        view.SetProcedure(procedure); // apply the procedure
        display.SetActive(true); // enable after procedure is applied
    }

    //This is used where we know the name of the procedure that will be opened (for example, the timeline summary)
    //It is used when that button is pressed to find and open the appropriate procedure
    public void UseProcedureButton(string procedureName)
    {
        CreateProcedureDisplay(FindProcedureByName(procedureName));
    }

    public bool OpenProcedureByName(string procedureName)
    {
        Procedure procedure = FindProcedureByName(procedureName);
        if (procedure == null)
        {
            Debug.LogWarning($"Procedures: No procedure found named '{procedureName}'.");
            return false;
        }

        CreateProcedureDisplay(procedure);
        return true;
    }

    private Procedure FindProcedureByName(string procedureName)
    {
        if (string.IsNullOrWhiteSpace(procedureName)) return null;

        List<BaseArsisEvent> currentProcedures = EventDatastore.Instance.GetEvents(key);
        foreach (BaseArsisEvent baseArsisEvent in currentProcedures)
        {
            if (baseArsisEvent is Procedure procedure &&
                procedure.data != null &&
                string.Equals(procedure.data.name, procedureName, StringComparison.OrdinalIgnoreCase))
                return procedure;
        }

        return null;
    }

    public void ShowSummaryTimeline()
    {
        if (noMenuChanges) return;
        if (summaryTimeline == null)
        {
            Debug.LogWarning("Procedures: summaryTimeline prefab is not assigned.");
            return;
        }

        FloatingMenuFromPrefab.OpenOrFocus(summaryTimeline);
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
        if (noMenuChanges) return;
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
