using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI; 
using TMPro; 

public class SettingsChangeEndpoint : MonoBehaviour
{
    private TextMeshProUGUI buttonText;
    public RawImage checkBox; 
    public UnityEvent action;
    public APICallHandler API;

    private bool state = true;

    float coolDown = 0f; // Cooldown exists because otherwise a button tap may be registered after the toggle

    private void Update()
    {
        if (coolDown >= 0)
        {
            coolDown -= Time.deltaTime;
        }
    }

    private void Start()
    {
        buttonText = GetComponentInChildren<TextMeshProUGUI>();
        if (buttonText != null) buttonText.text = API.GetEndpoint();
    }

    private bool canSelect()
    {
        return coolDown <= 0.1;
    }
    private void startCooldown()
    {
        coolDown = 1f;
    }

    public void toggle()
    {
        if (!canSelect()) return;
        startCooldown(); 
        action.Invoke(); 
        if (state)
        {
            state = false;
            if (buttonText != null)
            {
                API.SetEndpoint("localhost");
                buttonText.text = API.GetEndpoint();
            }
        } else
        {
            state = true;
            if (buttonText != null) 
            {
                API.SetEndpoint("bsusuits2022.herokuapp.com");
                buttonText.text = API.GetEndpoint();
            }
        }
    }
}
