using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ErrorManager : MonoBehaviour
{
    public static ErrorManager S;

    private void Start()
    {
        S = this;
    }

    public void fanError()
    {
        MenuController.s.currentProcedure = 1;
        MenuController.s.currentTask = 0;
        MenuController.s.currentSubTask = 0;
        VoiceManager.S.generateTaskMenu();
    }

    public void O2Error()
    {
        MenuController.s.currentProcedure = 2;
        MenuController.s.currentTask = 0;
        MenuController.s.currentSubTask = 0;
        VoiceManager.S.generateTaskMenu();
    }

    public void TemperatureError()
    {
        MenuController.s.currentProcedure = 3;
        MenuController.s.currentTask = 0;
        MenuController.s.currentSubTask = 0;
        VoiceManager.S.generateTaskMenu();
    }

    public void PressureError()
    {
        MenuController.s.currentProcedure = 4;
        MenuController.s.currentTask = 0;
        MenuController.s.currentSubTask = 0;
        VoiceManager.S.generateTaskMenu();
    }
}
