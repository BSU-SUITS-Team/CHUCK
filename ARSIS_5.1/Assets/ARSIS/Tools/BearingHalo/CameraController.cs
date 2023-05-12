using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    LocationCache locationCache;
    void Start()
    {
        locationCache = LocationCache.LocationCacheSingleton;
    }

    // Update is called once per frame
    void Update()
    {
        float headingAngle = locationCache.getHeading();
        transform.rotation = Quaternion.Euler(0.0f, headingAngle, 0.0f);
    }
}
