using System.Collections;
using System.Collections.Generic;
using System.Threading;
using ARSIS.EventManager;
using UnityEngine;
using System.Linq;
using MixedReality.Toolkit.UX;

public class SummaryTimeline : MonoBehaviour, IRenderable
{
    [SerializeField] RectTransform handle;
    [SerializeField] GameObject procedureDisplay;

    private const float cutoffInSeconds = 3300f;
    private float timerHeight = -250f;
    private EVA time;
    private List<BaseArsisEvent> procedures;
    private bool changed = true;
    private const string typeEVA = "eva";
    private const string typeProcedure = "procedure";


    void IRenderable.Render(List<BaseArsisEvent> data)
    {
        changed = true;
        BaseArsisEvent e = data.LastOrDefault();
        if (e is EVA newTime)
        {
            time = (EVA)e;
            return;
        }
        if (e is Procedure)
        {
            procedures = data;
            return;
        }
    }

    public void CreateProcedureDisplay(string name)
    {
        Procedure procedure = (Procedure)procedures.Where(procedure => procedure.label.Equals(name)).FirstOrDefault();
        if (procedure == null) return;
        GameObject display = Instantiate(procedureDisplay); // procedureDisplay prefab is active = false by default
        ProcedureDisplay view = display.GetComponent<ProcedureDisplay>();
        view.SetProcedure(procedure); // apply the procedure
        display.SetActive(true); // enable after procedure is applied
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(typeEVA, this);
        eventDatastore.AddHandler(typeProcedure, this);
    }

    void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(typeEVA, this);
        eventDatastore.RemoveHandler(typeProcedure, this);
    }

    void Update()
    {
        if (!changed) return;
        float newPos = Mathf.Clamp((time.data.total_time / cutoffInSeconds) * timerHeight, -250, 0);
        handle.anchoredPosition = new Vector2(handle.anchoredPosition.x, newPos);
        changed = false;
    }
}
