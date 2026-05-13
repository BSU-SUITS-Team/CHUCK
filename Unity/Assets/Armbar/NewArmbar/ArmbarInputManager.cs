using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//This script goes in the root of the scene and is used to determine what menu the user is looking at, 
//and if they're looking at one and press a button on their armbar, send the input to the ArmbarControllable on the menu
public class ArmbarInputManager : MonoBehaviour
{
    //A reference to the menu currently being controlled
    private ArmbarControllable controlledMenu;

    void Update()
    {
        CheckInput();
        CheckControlledMenu();
    }

    //Raycast out and update controlledMenu if we're looking at a new menu
    void CheckControlledMenu()
    {
        //Raycast out to see if we are looking at a menu
        RaycastHit hit;
        if (Physics.Raycast(Camera.main.transform.position, Camera.main.transform.TransformDirection(Vector3.forward), out hit, Mathf.Infinity))
        {
            //Check if the raycast hit object or one of its parents has ArmbarControllable, if so, update newControlled
            ArmbarControllable newControlled = null;
            if (hit.transform.GetComponent<ArmbarControllable>() != null) newControlled = hit.transform.GetComponent<ArmbarControllable>();
            else if (hit.transform.GetComponentInParent<ArmbarControllable>() != null) newControlled = hit.transform.GetComponentInParent<ArmbarControllable>();
            else Debug.LogWarning("RaycastHit object (" + hit.transform.gameObject.name + ") and parents don't have ArmbarControllable, so controlled menu won't be updated.");

            //If our raycast object or one if its parents has ArmbarControllable, update the menu being controlled, assuming it's not the one we're already controlling
            if (newControlled != null && newControlled != controlledMenu)
            {
                //If we were already controlling a menu, stop controlling it
                if (controlledMenu != null)
                {
                    //Hide all indicators
                    controlledMenu.ShowHideIndicators(false);
                    Debug.Log("Stopped controlling " + controlledMenu.gameObject.name + " and started controlling" + newControlled.gameObject.name);
                }
                else Debug.Log("Started controlling " + newControlled.gameObject.name);
                
                //Start controlling the new menu
                controlledMenu = newControlled;
                //Show all indicators
                controlledMenu.ShowHideIndicators(true);
            }
        }
    }

    //Check if the user is pressing down a button and if they are, pass it along to the menu being controlled
    void CheckInput()
    {
        //If we aren't controlling a menu, don't do anything
        if (controlledMenu == null) return;

        //Invoke events when a button is pressed. This is an else if list so we can't press two buttons on the same frame.
        if (Input.GetKeyDown("1")) controlledMenu.Button1Pressed.Invoke();
        else if (Input.GetKeyDown("2")) controlledMenu.Button2Pressed.Invoke();
        else if (Input.GetKeyDown("3")) controlledMenu.Button3Pressed.Invoke();
        else if (Input.GetKeyDown("4")) controlledMenu.Button4Pressed.Invoke();
        else if (Input.GetKeyDown("5")) controlledMenu.Button5Pressed.Invoke();
        else if (Input.GetKeyDown("6")) controlledMenu.Button6Pressed.Invoke();
    }
}
