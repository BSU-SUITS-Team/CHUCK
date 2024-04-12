using ARSIS.EventManager;
using MixedReality.Toolkit.Experimental;
using MixedReality.Toolkit.UX.Experimental;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScrollArea : MonoBehaviour
{
    [SerializeField] GameObject content;
    [SerializeField] InteractableEventRouter interactableEventRouter;
    private bool changed = true;
    private List<GameObject> entries = new();

    public void SetEntries(List<GameObject> entries)
    {
        this.entries = entries;
    }

    private void Clear()
    {
        foreach (Transform child in content.transform)
            Destroy(child.gameObject);
    }

    private void AddEntries()
    {
        foreach (GameObject entry in entries)
            entry.transform.SetParent(content.transform, false);
    }

    private void Update()
    {
        if (!changed) return;
        Clear();
        AddEntries();
        interactableEventRouter.Refresh();
        changed = false;
    }
}
