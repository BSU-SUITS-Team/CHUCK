using MixedReality.Toolkit.SpatialManipulation;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

/**
 * This script is to be placed at the root gameObject of the Window base prefab.
 */
public class Window : IBindings
{
    [SerializeField] GameObject main;
    [SerializeField] GameObject title;

    public void Close()
    {
        Destroy(gameObject);
    }

    public void SetContents(GameObject content)
    {
        Transform contentTransform = main.transform;
        content.transform.SetParent(contentTransform, false);
    }

    public void RemoveContents()
    {
        Transform contentTransform = main.transform;
        foreach (Transform transform in contentTransform)
        {
            Destroy(transform.gameObject);
        }
    }

    public void SetTitle(string title)
    {
        TextMeshProUGUI titleText = this.title.GetComponent<TextMeshProUGUI>();
        titleText.text = title;
    }
}
