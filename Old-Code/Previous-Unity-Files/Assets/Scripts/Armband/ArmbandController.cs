using ARSIS.UI;
using MixedReality.Toolkit.Input;
using MixedReality.Toolkit.UX;
using System;
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
    [SerializeField] GazeInteractor gazeInteractor;


    public static ArmbandController Instance { get; private set; }
    private GameObject currentView = null;
    private bool isListening = true;
    private KeyCode lastPressed = KeyCode.None;
    private Dictionary<KeyCode, (GameObject, GameObject)> views = new();
    private Dictionary<KeyCode, Action> bindings = null;

    // Start is called before the first frame update
    void Start()
    {
        if (Instance == null) Instance = this;
        views.Add(KeyCode.UpArrow, (upPrefab, null));
        views.Add(KeyCode.DownArrow, (downPrefab, null));
        views.Add(KeyCode.LeftArrow, (leftPrefab, null));
        views.Add(KeyCode.RightArrow, (rightPrefab, null));
        views.Add(KeyCode.Y, (yPrefab, null));
        views.Add(KeyCode.N, (nPrefab, null));
    }

    private void ToggleWindow(KeyCode key)
    {
        (GameObject prefab, GameObject view) = views[key];
        GameObject toggleView = view == null ? Instantiate(prefab) : null;
        if (toggleView == null) Destroy(view);
        views[key] = (prefab, toggleView);
    }

    public void SetListening(bool listening)
    {
        isListening = listening;
    }

    private Window FindWindow(GameObject go)
    {
        if (go == null) return null;
        Window window = go.GetComponent<Window>();
        if (window != null) return window;
        return FindWindow(go.transform.parent.gameObject);
    }

    private void SetLookingWindow() {
        var ray = new Ray(gazeInteractor.rayOriginTransform.position,
                      gazeInteractor.rayOriginTransform.forward * 3);
        if (Physics.Raycast(ray, out var hit))
        {
            Window window = FindWindow(hit.collider.gameObject);
            bindings = window.GetBindings();
            return;
        }
        bindings = null;
    }

    public void Execute(KeyCode key)
    {
        if (bindings.ContainsKey(key))
            bindings[key](); // execute binding of key
    }

    // Update is called once per frame
    void Update()
    {
        if (!isListening) return;

        SetLookingWindow();

        foreach (KeyCode key in views.Keys)
        {
            if (Input.GetKeyUp(key))
            {
                if (bindings != null)
                {
                    Execute(key);
                }
                else
                {
                    ToggleWindow(key);
                }
                return; // execute the first key found up
            }
        }
    }
}
