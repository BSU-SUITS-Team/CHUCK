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
    [SerializeField] GameObject endpointInput;

    private void Start()
    {
        TMP_InputField textArea = endpointInput.GetComponent<TMP_InputField>();
        EventManager instance = EventManager.Instance;
        textArea.text = instance.Endpoint;
    }

    public void SetEndpoint(string endpoint)
    {
        EventManager instance = EventManager.Instance;
        instance.Endpoint = endpoint;
    }

    public void RestartClient()
    {
        EventManager instance = EventManager.Instance;
        instance.EndClient();
        instance.StartClient();
    }
}
