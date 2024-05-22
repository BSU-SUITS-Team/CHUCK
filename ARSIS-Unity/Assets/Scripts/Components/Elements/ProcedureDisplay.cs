using ARSIS.EventManager;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class ProcedureDisplay : Window
{
    [SerializeField] GameObject previousButton;
    [SerializeField] GameObject nextButton;
    [SerializeField] TextMeshProUGUI taskName;
    [SerializeField] TextMeshProUGUI stepBody;

    private Dictionary<KeyCode, Action> bindings = null;
    private Procedure procedure;
    private int taskIndex = 0;
    private int stepIndex = 0;

    void Start()
    {
        bindings = new()
        {
            { KeyCode.LeftArrow, Previous },
            { KeyCode.RightArrow, Next },
        };
    }

    public override Dictionary<KeyCode, Action> GetBindings()
    {
        Debug.Log("new bindings");
        return bindings;
    }

    public void SetProcedure(Procedure procedure)
    {
        this.procedure = procedure;
        UpdateProcedure();
    }

    public void Previous()
    {
        if (!previousButton.activeSelf) return;
        int nextTask;
        int nextStep;
        if (stepIndex - 1 < 0)
        {
            nextTask = Mathf.Max(taskIndex - 1, 0);
            nextStep = procedure.data.tasks[nextTask].steps.Count - 1;
        }
        else
        {
            nextTask = taskIndex;
            nextStep = stepIndex - 1;
        }
        NavigateTask(nextTask, nextStep);
    }

    public void Next()
    {
        if (!nextButton.activeSelf) return;
        int nextTask;
        int nextStep;
        if (stepIndex + 1 >= procedure.data.tasks[taskIndex].steps.Count)
        {
            nextStep = 0;
            nextTask = Mathf.Min(taskIndex + 1, procedure.data.tasks.Count - 1);
        } 
        else
        {
            nextStep = stepIndex + 1;
            nextTask = taskIndex;
        }
        NavigateTask(nextTask, nextStep);
    }

    private void NavigateTask(int taskIndex, int stepIndex)
    {
        this.taskIndex = taskIndex;
        this.stepIndex = stepIndex;
        ProcedureTask currentTask = procedure.data.tasks[taskIndex];
        ProcedureStep currentStep = currentTask.steps[stepIndex];
        bool isFirst = taskIndex == 0 && stepIndex == 0;
        bool isLast = taskIndex + 1 >= procedure.data.tasks.Count && stepIndex + 1 >= currentTask.steps.Count;
        previousButton.SetActive(!isFirst);
        nextButton.SetActive(!isLast);
        taskName.text = currentTask.name;
        stepBody.text = currentStep.body;
    }

    private void UpdateProcedure()
    {
        Window window = gameObject.GetComponent<Window>();
        window.SetTitle(procedure.data.name);
        NavigateTask(0, 0);
    }
}
