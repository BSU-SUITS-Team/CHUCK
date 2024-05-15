using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using Unity.VisualScripting;
using UnityEngine;
using System.Linq;
using System;
using System.Numerics;
using Unity; 

public class Locations2 : MonoBehaviour, IRenderable
{

    [SerializeField] GameObject locationDisplay;
    private const string key = "imu";
    private const string key1 = "rover";
    private const string key2 = "pins";
    private List<BaseArsisEvent> locations = new List<BaseArsisEvent>(); //EVs
    private List<BaseArsisEvent> roverData = new List<BaseArsisEvent>();
    private List<BaseArsisEvent> pinsData = new List<BaseArsisEvent>();
    private bool changed = true;

    public void Render(List<BaseArsisEvent> data)
    {
        //locations = data; //storing list of locations from event system
        changed = true;
        if (data.FirstOrDefault() is IMU)
        {
            locations = data; 
            return; 
        }
        if (data.FirstOrDefault() is Rover)
        {
            roverData = data; 
            return; 
        }
        if (data.FirstOrDefault() is Pins)
        {
            pinsData = data; 
            return; 
        }
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(key, this); //imu 
        eventDatastore.AddHandler(key1, this); //rover
        eventDatastore.AddHandler(key2, this); //pins
    }

    void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(key, this); //imu
        eventDatastore.RemoveHandler(key1, this); //rover
        eventDatastore.RemoveHandler(key2, this); //pins

    }

    void Update()
    {
        if (!changed || locations.Count == 0) return; 
        changed = false; 

        IMU lastIMUData = (IMU)locations.LastOrDefault(); 
        if (lastIMUData != null) 
        {
            float eva1PosX = lastIMUData.data.eva1.posx; 
            float eva2PosX = lastIMUData.data.eva2.posx; 

            Debug.Log("Ev1 PosX: " + eva1PosX);
            Debug.Log("Ev2 PosX: " + eva2PosX);
        }

        
        // if (!changed || locations.Count == 0) return;

        // changed = false;
        // Debug.Log((IMU)locations[locations.Count -1]);
        // Debug.Log((Rover)roverData[roverData.Count -1]);
        // //Debug.Log((Pins)pinsData[pinsData.Count -1]);
        // Debug.Log((Pins)pinsData.LastOrDefault());

        // print(locations.LastOrDefault());

        // float originalPointEVX = locations.posx().LastOrDefault();
        
       //Vector2 originalPointEV = locations.LastOrDefault(); //Get EV's location

       //Vector3 virtualPoint = CoordinatesUtility.TranslateToVirtual(originalPointEV); //Use TranslateToVirtual
       //method to convert UTM coordinate of EV1
       //to Unity coordinate


        //UnityEngine.Vector3 virtualPoint = CoordinatesUtility.TranslateToVirtual(originalPoint);
    // Do stuff with virtual point

    }

}
