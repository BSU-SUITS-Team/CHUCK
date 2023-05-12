using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI; 
using TMPro; 

public class toggleButton : MonoBehaviour
{
    private TextMeshProUGUI buttonText;
    public RawImage checkBox; 
    public string startText;
    public string toggleText;
    public UnityEvent action;

    public Texture2D startImage;
    public Texture2D toggleImage;

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
        if (buttonText != null) buttonText.text = startText;

        //checkBox = GetComponentInChildren<RawImage>();
        if (checkBox != null) checkBox.texture = startImage; 
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
            if (buttonText != null) buttonText.text = toggleText;
            if (checkBox != null) checkBox.texture = toggleImage; 
        } else
        {
            state = true;
            if (buttonText != null) buttonText.text = startText;
            if (checkBox != null) checkBox.texture = startImage;
        }
    }
}
