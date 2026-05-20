using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using UnityEngine;
using UnityEngine.Events;

//This script goes in the root of the scene and is used to determine what menu the user is looking at, 
//and if they're looking at one and press a button on their armbar, send the input to the ArmbarControllable on the menu
public class ArmbarInputManager : MonoBehaviour, IRenderable
{
    private const string ArmbarButtonPressEventType = "armbar_button_press";

    //A reference to the menu currently being controlled
    private ArmbarControllable controlledMenu;
    private readonly object eventLock = new object();
    private List<BaseArsisEvent> armbarButtonPressEvents = new List<BaseArsisEvent>();
    private bool hasRemoteButtonPressChanges;
    private int processedRemoteButtonPressCount;
    private long lastHandledRemoteButtonPressTime;

    public UnityEvent dedicatedButton1Events, dedicatedButton2Events;

    void OnEnable()
    {
        lastHandledRemoteButtonPressTime = WebSocketClient.GetUnixTimeNanoseconds();
        EventDatastore.Instance.AddHandler(ArmbarButtonPressEventType, this);
    }

    void OnDisable()
    {
        EventDatastore.Instance.RemoveHandler(ArmbarButtonPressEventType, this);
    }

    public void Render(List<BaseArsisEvent> data)
    {
        lock (eventLock)
        {
            armbarButtonPressEvents = new List<BaseArsisEvent>(data);
            hasRemoteButtonPressChanges = true;
        }
    }

    void Update()
    {
        CheckInput();
        CheckControlledMenu();
        ProcessRemoteButtonPresses();
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
        if (Input.GetKeyDown("1")) controlledMenu.InvokeButton(1);
        else if (Input.GetKeyDown("2")) controlledMenu.InvokeButton(2);
        else if (Input.GetKeyDown("3")) controlledMenu.InvokeButton(3);
        else if (Input.GetKeyDown("4")) controlledMenu.InvokeButton(4);
        else if (Input.GetKeyDown("5")) controlledMenu.InvokeButton(5);
        else if (Input.GetKeyDown("6")) controlledMenu.InvokeButton(6);
        else if (Input.GetKeyDown("7")) dedicatedButton1Events.Invoke();
        else if (Input.GetKeyDown("8")) dedicatedButton2Events.Invoke();
    }

    private void ProcessRemoteButtonPresses()
    {
        List<BaseArsisEvent> events;

        lock (eventLock)
        {
            if (!hasRemoteButtonPressChanges)
                return;

            events = new List<BaseArsisEvent>(armbarButtonPressEvents);
            hasRemoteButtonPressChanges = false;
        }

        if (processedRemoteButtonPressCount > events.Count)
            processedRemoteButtonPressCount = 0;

        for (int i = processedRemoteButtonPressCount; i < events.Count; i++)
        {
            if (events[i] is not ArmbarButtonPress press)
                continue;

            if (press.time <= lastHandledRemoteButtonPressTime)
                continue;

            int button = press.data != null ? press.data.button : 0;
            if (button < 1 || button > 6)
            {
                Debug.LogWarning($"ArmbarInputManager: Ignoring unsupported armbar button '{button}'.");
                continue;
            }

            InvokeButton(button);
            lastHandledRemoteButtonPressTime = press.time;
        }

        processedRemoteButtonPressCount = events.Count;
    }

    private void InvokeButton(int button)
    {
        if (controlledMenu == null)
        {
            Debug.LogWarning($"ArmbarInputManager: No controlled menu is available for remote button {button}.");
            return;
        }

        controlledMenu.InvokeButton(button);
    }

    public void CloseAllWindows()
    {
        Window[] windows = GameObject.FindObjectsOfType<Window>();
        foreach (Window window in windows) window.Close();
    }
}
