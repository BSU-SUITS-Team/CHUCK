using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using Unity.VisualScripting;
using UnityEngine;
using System.Linq;
using System;
using System.Numerics;
using Unity;
using MixedReality.Toolkit.Input;
using JetBrains.Annotations;



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
    private static float eva1PosX;
    private static float eva1PosY; 


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
            float eva1PosY = lastIMUData.data.eva1.posy; 

            float eva2PosX = lastIMUData.data.eva2.posx;
            float eva2PosY = lastIMUData.data.eva2.posy;

            Debug.Log("Ev1 PosX: " + eva1PosX);
            Debug.Log("Ev1 PosY: " + eva1PosY);

            Debug.Log("Ev2 PosX: " + eva2PosX);
            Debug.Log("Ev2 PosY: " + eva2PosY);

            eva1PosX = 298355; //Middle of map, test scenario
            eva1PosY = 3272383;//Middle of map, test scenario 

            UnityEngine.Vector2 ev1PosUTM = new UnityEngine.Vector2(eva1PosX, eva1PosY);

            UnityEngine.Vector3 ev1PosUnity = CoordinatesUtility.TranslateToVirtual(ev1PosUTM);

            Debug.Log("EV1PosUnity: " + ev1PosUnity);

        }

        Rover lastRoverData = (Rover)roverData.LastOrDefault();
        if (lastRoverData != null)
        {
            //float roverPosX = lastRoverData.data.rover.posx; 
            //float roverPosY = lastRoverData.data.rover.posy; 

            //Debug.Log("Rover PosX: " + roverPosX);
            //Debug.Log("Rover PosY: " + roverPosY);
        }

        Pins lastPinsData = (Pins)pinsData.LastOrDefault();
        if (lastPinsData != null)
        {
            int pinPosX = lastPinsData.data.properties.x;
            int pinPosY = lastPinsData.data.properties.y;

            Debug.Log("Pin PosX: " + pinPosX);
            Debug.Log("Pin PosY: " + pinPosY);

            UnityEngine.Vector2 pinPosUTM = new UnityEngine.Vector2(pinPosX, pinPosY);
            UnityEngine.Vector3 pinPosUnity = CoordinatesUtility.TranslateToVirtual(pinPosUTM);

            Debug.Log("PinPosUnity: " + pinPosUnity);

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
