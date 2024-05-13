using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using Unity.VisualScripting;
using UnityEngine;

public class Locations2 : MonoBehaviour, IRenderable
{

    [SerializeField] GameObject locationDisplay;
    private const string key = "imu";
    private const string key1 = "rover";
    private List<BaseArsisEvent> locations = new List<BaseArsisEvent>();
    private bool changed = true;

    public void Render(List<BaseArsisEvent> data)
    {
        locations = data; //storing list of locations from event system
        changed = true;
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(key, this); //imu 
        eventDatastore.AddHandler(key1, this); //rover
    }

    void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(key, this);//imu
        eventDatastore.RemoveHandler(key1, this);//rover
    }

    void Update()
    {
        if (!changed || locations.Count == 0) return;
        // List<GameObject> entries = new();
        // foreach (BaseArsisEvent baseArsisEvent in locations)
        // {
        //     if (baseArsisEvent is Location location)
        //     {
        //         Button button = entry.GetComponent<Button>();
        //         button.SetText(location.data.name);
        //         PressableButton pressableButton = button.GetPressableButton();
        //         pressableButton.OnClicked.AddListener(() => CreateLocationDisplay(location));
        //         entries.Add(entry);
        //     }
        // }
        // scrollArea.SetEntries(entries);
        changed = false;
        Debug.Log(((IMU)locations[locations.Count -1]));
        //Debug.Log(((IMU)locations[locations.Count -1]).data.imu);
        
    }


}
