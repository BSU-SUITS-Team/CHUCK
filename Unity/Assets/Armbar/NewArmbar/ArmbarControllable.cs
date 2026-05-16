using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using MixedReality.Toolkit.SpatialManipulation;
using MixedReality.Toolkit.UX;
using MixedReality.Toolkit;

//This script goes on windows (such as the Biometrics window) to tell it what to do when a particular button is pressed
//Typically, Button1 is used for closing the window and Button2 is used for toggling window following

//To set up a new ArmbarControllable window or update it:
// - Ensure the root of the window (which should have the Window and Follow components) has this script on it
// - There are prefabs in Assets/Resources/ArmbarControllerAssets/Prefabs to indicate which what pressing different buttons will do, place
//   these by the buttons they are controlling
// - In the inspector for this script, add all of the indicators you just created to the indicators list
// - In the inspector for this script, add UnityEvents for each button you're using, these should mimic the UnityEvents on the buttons the
//   indicators are next to

public class ArmbarControllable : MonoBehaviour
{
    //Should we ignore arm bar inputs?
    public bool locked {get; set;}

    [Tooltip("A list of all button indicators on this menu. This is used to show or hide indicators when the user starts or stops looking at the window.")]
    public List<GameObject> indicators = new List<GameObject>();

    [Header("Button Presses")]
    [Tooltip("When this button on the armbar is pressed, what should be called?")]
    //Button1Pressed is on a different line so the header doesn't get repeated
    public UnityEvent Button1Pressed = new UnityEvent();
    [Tooltip("When this button on the armbar is pressed, what should be called?")]
    public UnityEvent Button2Pressed, Button3Pressed, Button4Pressed, Button5Pressed, Button6Pressed = new UnityEvent();

    private void Awake()
    {
        //Hide all indicators before the first frame with this menu is shown
        ShowHideIndicators(false);
    }

    //Toggle whether the indicators are visible. This is called by ArmbarInputManager when the menu being controlled changes.
    public void ShowHideIndicators(bool show)
    {
        foreach(GameObject indicator in indicators) indicator.SetActive(show);
    }

    public void ActivateMRTKButton(PressableButton button)
    {
        if (button == null)
            return;

        StatefulInteractable interactable = button;

        bool current = interactable.IsToggled;
        interactable.ForceSetToggled(!current);
     
        button.OnClicked.Invoke();
    }

    public void InvokeButton(int i)
    {
        if (!locked)
        {
            switch (i)
            {
                case 1:
                    Button1Pressed.Invoke();
                    break;
                case 2:
                    Button2Pressed.Invoke();
                    break;
                case 3:
                    Button3Pressed.Invoke();
                    break;
                case 4:
                    Button4Pressed.Invoke();
                    break;
                case 5:
                    Button5Pressed.Invoke();
                    break;
                case 6:
                    Button6Pressed.Invoke();
                    break;
                default:
                    break;
            }
        }
    }
}
