using ARSIS.EventManager;
using ARSIS.UI;
using MixedReality.Toolkit.Experimental;
using MixedReality.Toolkit.UX.Experimental;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Procedures : MonoBehaviour, IRenderable
{
    [SerializeField] GameObject content;
    [SerializeField] VirtualizedScrollRectList procedureList; // has the VirtualizedScrollRectList
    [SerializeField] GameObject procedureButton;
    [SerializeField] InteractableEventRouter interactableEventRouter;
    private static string key = "procedure";
    private List<BaseArsisEvent> procedures = new List<BaseArsisEvent>();
    private bool changed = true;

    void IRenderable.Render(List<BaseArsisEvent> list)
    {
        procedures = list;
        changed = true;
        //procedureList.SetItemCount(procedures.Count);
    }

    void ConfigureButton(GameObject prefab, int index)
     {
        Button button = prefab.GetComponent<Button>();
        if (button == null || procedures == null || procedures[index] == null) return;
        if (procedures[index] is Procedure)
        {
            Procedure procedureEvent = (Procedure)procedures[index];
            button.SetText(procedureEvent.label);
        } else
        {
            Debug.LogError("Procedures: Not a procedure!!");
        }
        //Transform transform = prefab.transform;
        //transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.Euler(Vector3.zero));
    }

    void Update()
    {
        if (changed)
        {
            foreach (Transform child in content.transform)
            {
                Destroy(child.gameObject);
            }
            foreach (BaseArsisEvent baseArsisEvent in procedures)
            {
                if (baseArsisEvent is Procedure)
                {
                    GameObject entry = Instantiate(procedureButton);
                    entry.transform.SetParent(content.transform, false);
                    Button button = entry.GetComponent<Button>();
                    button.SetText(baseArsisEvent.label);
                }
            }
            interactableEventRouter.Refresh();
            changed = false;
        }
    }

    void Start()
    {
        //procedureList.SetItemCount(0);
        //procedureList.OnVisible = ConfigureButton;
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(key, this);
    }

    private void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(key, this);
    }
}
