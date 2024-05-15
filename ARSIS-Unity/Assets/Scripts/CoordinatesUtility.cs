using System.Collections;
using ARSIS.EventManager;
using Unity.VisualScripting;
using UnityEngine;
using System.Linq;
using System;
using System.Numerics;
using Unity;
using MixedReality.Toolkit;

public class CoordinatesUtility
{
    public UnityEngine.Vector3 translatedPoint { get; private set; }

    // public int GetCoordinateVector(int originalPoint) {
        
    //     return originalPoint;  
    //     //need to know what type the coordinate is 
    // }

    public UnityEngine.Vector3 TranslateToVirtual(System.Numerics.Vector2 originalPoint)//Converting UTM coordinates to Unity coordinates
    {
        //get most recent coordinate of ev1
        
        // Do translation stuff here
        return translatedPoint; 
    }

}



