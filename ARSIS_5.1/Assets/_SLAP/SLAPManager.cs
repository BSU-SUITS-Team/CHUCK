using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SLAPManager : MonoBehaviour
{
    public static SLAPManager instance;

    public SLAPAstronautHandler astronaut;
    public APICallHandler ach;

    void Awake ()
    {
        instance = this;
    }

    void Start()
    {
        ach = APICallHandler.instance;
        astronaut = SLAPAstronautHandler.instance;
    }

    public async void CalibratePosition()
    {
        SLAPUtils.astronaut = astronaut;
        await SLAPUtils.CallibrateStartPosAsync();    // switch this to initial calibration button
    }
}
