using ARSIS.EventManager;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class Notifications : MonoBehaviour
{
    [SerializeField] TextMeshProUGUI connectionText;

    void Update()
    {
        EventManager eventManager = EventManager.Instance;
        string status = eventManager.Client.GetStatus();
        connectionText.text = "Connection: " + status;
    }
}
