using ARSIS.EventManager;
using MixedReality.Toolkit.UX;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
//using MixedReality.Toolkit.SpatialManipulation;

/**
 * This script is to be placed at the root gameObject of the SettingsWindow variant prefab.
 */
public class SettingsWindow : MonoBehaviour
{
    [SerializeField] MRTKUGUIInputField inputField;
    private TouchScreenKeyboard keyboard;

    public void OpenSystemKeyboard()
    {
        keyboard = TouchScreenKeyboard.Open(inputField.text, TouchScreenKeyboardType.URL, false, false, false, false);
        ArmbandController.Instance.SetListening(false);
    }

    void Start()
    {
        EventManager instance = EventManager.Instance;
        inputField.text = instance.Endpoint;
        //StartCoroutine(StopFollow());
    }

    // private IEnumerator StopFollow()
    // {
    //     yield return new WaitForSeconds(0.1f);
    //     Follow followObject = GetComponent<Follow>();
    //     if (followObject != null)
    //     {
    //         followObject.enabled = false;
    //     }
    // }

    private void OnDestroy()
    {
        ArmbandController.Instance.SetListening(true);
    }

    public void SetEVA(int eva)
    {
        EventManager instance = EventManager.Instance;
        instance.Eva = eva;
        EventDatastore.Instance.NotifyAll();

        TSSConnectionManager.Instance?.SetEva(eva);
    }

    public void SetEndpoint()
    {
        EventManager instance = EventManager.Instance;

        string value = inputField.text.Trim();

        if (value.StartsWith("ws://") || value.StartsWith("wss://"))
        {
            instance.Endpoint = value;
        }
        else
        {
            instance.Endpoint = $"ws://{value}:8181/ws/events";
        }
    }

    public void RestartClient()
    {
        EventManager instance = EventManager.Instance;
        instance.EndClient();
        instance.StartClient();
    }

    private void Update()
    {
        if (keyboard == null) return;
        inputField.text = keyboard.text;
        if (keyboard.status == TouchScreenKeyboard.Status.Done)
        {
            keyboard = null;
            ArmbandController.Instance.SetListening(true);
        }
    }
}
