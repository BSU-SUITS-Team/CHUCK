using ARSIS.EventManager;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class Notifications : MonoBehaviour
{
    [SerializeField] TextMeshProUGUI connectionText;
    [SerializeField] TextMeshProUGUI evaText;

    void Update()
    {
        EventManager eventManager = EventManager.Instance;
        string status = eventManager.Client.GetStatus();
        connectionText.text = status;
        int eva = eventManager.Eva;
        evaText.text = "EVA" + eva;
    }
}
