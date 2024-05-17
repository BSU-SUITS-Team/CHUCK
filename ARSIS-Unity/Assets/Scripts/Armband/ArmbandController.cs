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
    private bool isListening = true;
    private Dictionary<KeyCode, (GameObject, GameObject)> views = new();

    // Start is called before the first frame update
    void Start()
    {
        if (Instance == null) Instance = this;
        views.Add(KeyCode.UpArrow, ( upPrefab, null ));
        views.Add(KeyCode.DownArrow, ( downPrefab, null ));
        views.Add(KeyCode.LeftArrow, ( leftPrefab, null ));
        views.Add(KeyCode.RightArrow, ( rightPrefab, null ));
        views.Add(KeyCode.Y, ( yPrefab, null ));
        views.Add(KeyCode.N, ( nPrefab, null ));
    }

    private void ToggleWindow()
    {
        foreach (KeyCode key in views.Keys)
        {
            if (Input.GetKeyUp(key))
            {
                (GameObject prefab, GameObject view) = views[key];
                GameObject toggleView = view == null ? Instantiate(prefab) : null;
                if (toggleView == null) Destroy(view);
                views[key] = (prefab, toggleView);
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
