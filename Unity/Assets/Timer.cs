using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System;

public class Timer : MonoBehaviour
{
    public TextMeshProUGUI text;
    float t;
    private bool timerRunning;
    
    void Update()
    {
        if (!timerRunning) return;
        
        t += Time.deltaTime;
        TimeSpan time = TimeSpan.FromSeconds(t);
        text.text = "Timer: " + time.ToString("hh':'mm':'ss");
    }

    public void StartStopTimer()
    {
        if (timerRunning){
            timerRunning = false;
            t = 0;
        }
        else timerRunning = true;
    }
}
