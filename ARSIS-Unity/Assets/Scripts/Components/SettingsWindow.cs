using ARSIS.EventManager;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

/**
 * This script is to be placed at the root gameObject of the SettingsWindow variant prefab.
 */
public class SettingsWindow : MonoBehaviour
{
    private GameObject text;

    void Start()
    {
        text = gameObject.transform.Find("/Plate/Container/Main/Vertical/Header/EndpointInput").gameObject;
    }

    void SetWebSocket()
    {
        TMP_InputField textArea = text.GetComponent<TMP_InputField>();
        string endpoint = textArea.text;
        EventManager instance = EventManager.Instance;
        instance.Endpoint = endpoint;
        instance.EndClient();
        instance.StartClient();
    }
}
