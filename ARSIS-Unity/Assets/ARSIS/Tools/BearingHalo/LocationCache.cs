using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

public class LocationCache : MonoBehaviour
{

    public LocationEvent locationEvent;
    private static int MINUTES = 5;
    private static int MAX_ENTRYS = 60*MINUTES;
    private List<LocationEvent> LocationList;
    public static LocationCache LocationCacheSingleton { get; private set; }
    private void Awake()
    {
        LocationList = new List<LocationEvent>();
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
         if (LocationList.Count > MAX_ENTRYS){
            LocationList.RemoveAt(0);
        }
        LocationList.Add(he);
        Debug.Log(LocationList.Count);
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
    public List<float> getLocationList{
        List<float> locationList = new List<float>()
        foreach (LocationEvent locationEvent in LocationList){
            locationList.append(locationEvent.location);
        }
        return locationList;
    }
}
