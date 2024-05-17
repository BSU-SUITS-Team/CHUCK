using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using Unity.VisualScripting;
using UnityEngine;
using System.Linq;
using System;
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
    private ImuEva evaData;



   private void RetrieveIMU(int eva)
    {
        IMU imu = (IMU)locations.Last();
        evaData = eva switch
        {
            2 => imu.data.eva2, // eva2
            _ => imu.data.eva1, // default or eva1
        };
    }
   
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
        // y is northing, x is easting (Vector2)
        // z is forward, y is height, x is right (Vector3)
        Vector2 a = new Vector2(0, 1); // test origin
        Vector2 b = new Vector2(0, 1); // test coordinate
        float testBearing = 0f; // test bearing
        for (int i = 0; i < 5; i++) {
            Vector3 testOutput = CoordinatesUtility.TranslateToVirtual(b, a, testBearing);
            Debug.Log($"bearing: {testBearing}, testOutput: {testOutput}"); // expect 1 meter in front of camera
            testBearing += 45f;
        }

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

        //Look at biometrics script

        if (!changed || locations.Count == 0) return; 
        changed = false; 

        EventManager eventManager = EventManager.Instance;
        RetrieveIMU(eventManager.Eva);

        IMU IMUData = (IMU)locations.LastOrDefault(); //EV1 and EV2
        if (IMUData != null) 
        {
            float userPosX = evaData.posx; 
            float userPosY = evaData.posy; 
            float userHeading = evaData.heading;  

            Debug.Log("Ev1 PosX: " + userPosX);
            Debug.Log("Ev1 PosY: " + userPosY);
            Debug.Log("EV1 Heading: " + userHeading);

            userPosX = 298355; //Middle of map, test scenario
            userPosY = 3272383;//Middle of map, test scenario 

            Vector2 userPosUTM = new Vector2(userPosX, userPosY);
            //float ev1Heading = CoordinatesUtility.CalculateHeading(ev1PosUTM, UTMorigin);

           // Vector3 ev1PosUnity = CoordinatesUtility.TranslateToVirtual(selectedPin, userPosUTM, userHeading);

            //Debug.Log("EV1PosUnity: " + ev1PosUnity);

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

            Vector2 pinPosUTM = new Vector2(pinPosX, pinPosY);
            //Vector3 pinPosUnity = CoordinatesUtility.TranslateToVirtual(pinPosUTM);

            //Debug.Log("PinPosUnity: " + pinPosUnity);

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
