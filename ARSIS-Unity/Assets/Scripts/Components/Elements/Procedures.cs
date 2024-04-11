using ARSIS.EventManager;
using ARSIS.UI;
using MixedReality.Toolkit.UX.Experimental;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Procedures : MonoBehaviour, IRenderable
{
    [SerializeField] GameObject content; // has the VirtualizedScrollRectList
    [SerializeField] GameObject procedureButton;
    private List<BaseArsisEvent> procedures;

    void IRenderable.Render(List<BaseArsisEvent> list)
    {
        Debug.Log("get udpate");
        procedures = list;
        Transform contentTransform = content.transform;
        foreach (Transform transform in contentTransform)
        {
            Destroy(transform.gameObject);
        }
        for (int i = 0; i < procedures.Count; i++)
        {
            GameObject button = Instantiate(procedureButton);
            button.transform.SetParent(contentTransform, false);
            Button buttonText = button.GetComponent<Button>();
            buttonText.SetIcon(false, "", "");
            buttonText.SetText(enabled, "Procedure " + i);
        }
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler("pins", this);
    }

    private void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler("pins", this);
    }
}
