using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MixedReality.Toolkit.UX;

public class ArmbarButtonList : MonoBehaviour
{
    //A list of each PressableButton in the list. Gets populated with all PressableButton children in buttonListParent on Awake.
    private List<PressableButton> buttons = new List<PressableButton>();

    //The object that contains the list of all PressableButtons
    public GameObject buttonListParent;

    //The parent object of the scroll up, scroll down and interact indicators
    public GameObject scrollAndPressIndicators;

    //The scroll up and scroll down indicators
    public GameObject scrollUpIndicator, scrollDownIndicator;

    private int index;

    private void Awake()
    {
        //Populate a list of PressableButtons based on the children of buttonListParent
        foreach(PressableButton pb in buttonListParent.GetComponentsInChildren<PressableButton>()) buttons.Add(pb);
    }

    public void MoveDown()
    {
        if (index < buttons.Count - 1) index ++;
        UpdateIndicators();
    }

    public void MoveUp()
    {
        if (index > 0) index --;
        UpdateIndicators();
    }

    public void PressSelectedButton()
    {
        buttons[index].OnClicked.Invoke();
    }

    private void UpdateIndicators()
    {
        //Move the indicators to be next to the associated button
        scrollAndPressIndicators.transform.position = buttons[index].transform.position;

        //If we're at the first or last item in the list, hide the scroll up or down indicator
        if (index == 0) scrollUpIndicator.SetActive(false);
        else scrollUpIndicator.SetActive(true);

        if (index == buttons.Count - 1) scrollDownIndicator.SetActive(false);
        else scrollDownIndicator.SetActive(true);
    }
}
