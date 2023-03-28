using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using EventSystem;

public class LocationCache : MonoBehaviour
{

    public LocationEvent locationEvent;
    public static LocationCache LocationCacheSingleton { get; private set; }
    private void Awake()
    {
        if (LocationCacheSingleton != null && LocationCacheSingleton != this)
        {
            Destroy(this);
            EventManager.RemoveListener<LocationEvent>(UpdateLocation);
        }
        else
        {
            LocationCacheSingleton = this;
            EventManager.AddListener<LocationEvent>(UpdateLocation);
        }
    }
    void UpdateLocation(LocationEvent he){
        locationEvent = he;
        Debug.Log(locationEvent);
    }

    public float getHeading(){
        float heading = 0;
        if(locationEvent != null){
            heading = locationEvent.heading;
        }
        Debug.Log(heading);
        Debug.Log(locationEvent);
        return heading;
    }

    public string getLocationString(){
        return locationEvent.heading.ToString();
    }
}
