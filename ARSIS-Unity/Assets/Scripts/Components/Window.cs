using MixedReality.Toolkit.SpatialManipulation;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

/**
 * This script is to be placed at the root gameObject of the Window base prefab.
 */
public class Window : MonoBehaviour
{
    private GameObject main;
    private GameObject title;

    void Start()
    {
        main = gameObject.transform.Find("/Plate/Container/Main").gameObject;
        title = gameObject.transform.Find("/Plate/Container/Header/Title").gameObject;
        Follow follow = gameObject.GetComponent<Follow>();
        follow.enabled = true; // to place the game object at the user
        follow.enabled = false; // disable follow, managed by pin button
    }

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
