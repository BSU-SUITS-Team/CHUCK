using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SLAPMissionObject : MonoBehaviour
{
    public SLAPUtils.LatLon latlon;

    public void UpdatePosition ()
    {
        transform.position = SLAPUtils.LatLonToUnityCoords(latlon);
    }
}
