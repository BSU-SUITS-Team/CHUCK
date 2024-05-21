using ARSIS.UI;
using MixedReality.Toolkit.UX;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

public class ArmbandController : MonoBehaviour
{
    [SerializeField] GameObject upPrefab;
    [SerializeField] GameObject downPrefab;
    [SerializeField] GameObject leftPrefab;
    [SerializeField] GameObject rightPrefab;
    [SerializeField] GameObject yPrefab;
    [SerializeField] GameObject nPrefab;


    public static ArmbandController Instance { get; private set; }
    private GameObject currentView = null;
    private bool isListening = true;
    private KeyCode lastPressed = KeyCode.None;
    private Dictionary<KeyCode, GameObject> views = new();

    // Start is called before the first frame update
    void Start()
    {
        if (Instance == null) Instance = this;
        views.Add(KeyCode.UpArrow, upPrefab);
        views.Add(KeyCode.DownArrow, downPrefab);
        views.Add(KeyCode.LeftArrow, leftPrefab);
        views.Add(KeyCode.RightArrow, rightPrefab);
        views.Add(KeyCode.Y, yPrefab);
        views.Add(KeyCode.N, nPrefab);
    }

    private void ToggleWindow()
    {
        foreach (KeyCode key in views.Keys)
        {
            if (Input.GetKeyUp(key))
            {
                if (lastPressed == KeyCode.None){
                    currentView = Instantiate(views[key]);
                    lastPressed = key;
                    return;
                }
                Destroy(currentView);
                currentView = lastPressed == key ? null : Instantiate(views[key]);
                lastPressed = lastPressed == key ? KeyCode.None : key;
                return;
            }
        }
    }

    public void SetListening(bool listening)
    {
        isListening = listening;
    }

    // Update is called once per frame
    void Update()
    {
        if (!isListening) return;

        // if eye gaze on window
        //     invoke window buttons based on window keymap
        //     return

        // toggle views if eyegaze not on window
        ToggleWindow();
    }
}
