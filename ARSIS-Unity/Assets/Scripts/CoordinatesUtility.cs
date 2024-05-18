using System.Collections;
using ARSIS.EventManager;
using Unity.VisualScripting;
using UnityEngine;
using System.Linq;
using System;
using Unity;
using MixedReality.Toolkit;
using UnityEngine.InputSystem.Interactions;


public static class CoordinatesUtility
{
    //public static UnityEngine.Vector2 UTMorigin = new UnityEngine.Vector2(32, 463); //(A,0) needs switched from lat,long to UTM
    public static UnityEngine.Vector2 UTMorigin = new UnityEngine.Vector2(298305, 3272330); //(A,0)

    public static UnityEngine.Vector3 translatedPoint { get; private set; }

    //Calculates new heading/position in Unity
    public static UnityEngine.Vector3 TranslateToVirtual(UnityEngine.Vector2 selectedPin, UnityEngine.Vector2 user, float userBearing) //Converting UTM coordinates to Unity coordinates
    {
        UnityEngine.Vector3 cameraPos = Camera.main.transform.position; //Unity coordinates
        Debug.Log("Camera Position: " + cameraPos);
        
        UnityEngine.Vector2 delta = selectedPin - user; 
        float distance = delta.magnitude; //Calculate distance
        Debug.Log($"selectedPin: {selectedPin}, user: {user}");
        Debug.Log($"distance: {distance}");

        //Find heading of the user to selectedPin
        float bearingRad = Mathf.Atan2(delta.x, delta.y);// use deltaX and deltaY
        float bearingDeg = bearingRad*Mathf.Rad2Deg; 

        //Convert angle to range of 0 to 360 degrees
        float deltaBearing = (bearingDeg + 360.0f) % 360.0f; 
        float correctedDeltaBearing = bearingRad - (userBearing * Mathf.Deg2Rad);

        Vector3 virtualOffset = new Vector3(distance*Mathf.Sin(correctedDeltaBearing), 0, distance*Mathf.Cos(correctedDeltaBearing));
        Debug.Log($"virtualOffset: {virtualOffset}");
        Vector3 translatedPoint = virtualOffset + cameraPos;

        Debug.Log($"translatedPoint: {translatedPoint}");

        return translatedPoint;
    }

    //public static double CalculateHeading(UnityEngine.Vector2 selectedPin, UnityEngine.Vector2 user, float userHeading)
   // {
        // UnityEngine.Vector2 delta = selectedPin - user; 
        // float angleRad = Mathf.Atan2(delta.y, delta.x);//use deltaX and deltaY
        // //Calculate distance 
        // float distance = delta.magnitude; 
        // float angleDeg = angleRad*Mathf.Rad2Deg; 

        // //Convert angle to range of 0 to 360 degrees
        // float compassHeading = (angleDeg + 360.0f) % 360.0f; 
        // float correctedCompassHeading = compassHeading - userHeading; 

        // return compassHeading; 
    //}

}



