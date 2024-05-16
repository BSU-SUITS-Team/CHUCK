using System.Collections;
using ARSIS.EventManager;
using Unity.VisualScripting;
using UnityEngine;
using System.Linq;
using System;
using System.Numerics;
using Unity;
using MixedReality.Toolkit;


public static class CoordinatesUtility
{
    public static UnityEngine.Vector2 UTMorigin = new UnityEngine.Vector2(32, 463); //(A,0) needs switched from lat,long to UTM
    public static UnityEngine.Vector3 translatedPoint { get; private set; }

    // public int GetCoordinateVector(int originalPoint) {
        
    //     return originalPoint;  
    //     //need to know what type the coordinate is 
    // }

    public static UnityEngine.Vector3 TranslateToVirtual(UnityEngine.Vector2 originalPoint) //Converting UTM coordinates to Unity coordinates
    {
        UnityEngine.Vector2 localUTM = originalPoint - UTMorigin; 

        float unityX = localUTM.x; 
        float unityZ = localUTM.y; 
        
        // Do translation stuff here
        return translatedPoint = new UnityEngine.Vector3(unityX, 0, unityZ);
    }

}



