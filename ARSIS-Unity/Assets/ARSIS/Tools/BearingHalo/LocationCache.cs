using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

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
    }

    public float getHeading(){
        float heading = 0;
        if(locationEvent != null){
            heading = locationEvent.heading;
        }
        return heading;
    }

    public string getLocationString(){
        return locationEvent.heading.ToString();
    }
}
