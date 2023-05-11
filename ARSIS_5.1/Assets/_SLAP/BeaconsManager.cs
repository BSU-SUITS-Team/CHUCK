using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class BeaconsManager : MonoBehaviour
{
    #region data structures
    public enum beacon_type_name
    {
        lunasar,
        general
    }

    [Serializable]
    public class BeaconType
    {
        public beacon_type_name type;
        public GameObject prefab;
        public Sprite map_icon;
    }

    [Serializable]
    public class Beacon
    {
        public GameObject gameobject;
        public beacon_type_name type;
        public SLAPMissionObject mission_object;

        public Beacon (GameObject go, beacon_type_name type, SLAPUtils.LatLon latlon, bool is_main)
        {
            this.gameobject = go;
            this.type = type;
            this.mission_object = go.GetComponent<SLAPMissionObject>();
            this.mission_object.latlon = latlon;
            this.gameobject.transform.position = SLAPUtils.LatLonToUnityCoords(latlon);

            // Compass.instance.needles.Add(new Compass.PointerObject(mission_object, is_main));
        }
    }
    #endregion

    #region fields
    public static BeaconsManager instance;
    //public MapManager map_manager;
    private SLAPUtils.LatLon[] initial_beacon_locations =
    {
        new SLAPUtils.LatLon(
            29.522051,
            -95.1221484
        ),
        new SLAPUtils.LatLon(
            29.523791,
            -95.1227442
        ),
        new SLAPUtils.LatLon(
            29.521743,
            -95.1221308
        ),
        new SLAPUtils.LatLon(
            29.529072,
            -95.1220769
        ),
        new SLAPUtils.LatLon(
            29.528872,
            -95.1220565
        ),
        new SLAPUtils.LatLon(
            29.5242656,
            -95.1223498
        ),
        /*
        // test day beacon locations
        new SLAPUtils.LatLon(
            29.5648150,
            -95.0817410
        ),
        new SLAPUtils.LatLon(
            29.5646824,
            -95.0811564
        ),
        new SLAPUtils.LatLon(
            29.5650460,
            -95.0810944
        ),
        new SLAPUtils.LatLon(
            29.5645430,
            -95.0516440
        ),
        new SLAPUtils.LatLon(
            29.5648290,
            -95.0813750
        ),
        new SLAPUtils.LatLon(
            29.5647012,
            -95.0813750
        ),
            new SLAPUtils.LatLon(
            29.5651359,
            -95.0807408
        ),
        new SLAPUtils.LatLon(
            29.5651465,
            -95.0814092
        ),
        new SLAPUtils.LatLon(
            29.5648850,
            -95.0808360
        )
        */
    };

    public BeaconType[] types;
    public List<Beacon> curr_beacons;
    #endregion

    #region methods
    public void SpawnBeacon ()
    {
        SpawnBeacon(new SLAPUtils.LatLon(0, 0), beacon_type_name.general, false);
    }
    #endregion

    #region methods
    void Awake ()
    {
        instance = this;
    }

    public void SpawnInitialBeacons ()
    {
        foreach (SLAPUtils.LatLon ll in initial_beacon_locations)
        {
            SpawnBeacon(ll, beacon_type_name.general, false);
        }
    }

    public void SpawnBeacon (SLAPUtils.LatLon lat_long, beacon_type_name type, bool is_main)
    {
        BeaconType beacon_type = null;
        foreach (BeaconType t in types)
        {
            if (type.Equals(t.type))
            {
                beacon_type = t;
            }
        }

        GameObject beacon_go = Instantiate(beacon_type.prefab, gameObject.transform);
        Beacon b = new Beacon(beacon_go, beacon_type_name.general, lat_long, is_main);


        curr_beacons.Add(b);

        /*
        Beacon b = new Beacon();
        BeaconType bt = null;

        // set prefab to spawn based on beacontype
        foreach (BeaconType t in types)
        {
            if (type.Equals(t.type))
            {
                //prefab = t.prefab;
                bt = t;
            }
        }
        
        // break out if there isn't there isn't enough info to do what we need
        if (bt == null) {
            return;
        }

        // create a beacon and add it to the list of current beacons
        b.type = type;
        b.lat_long = lat_long;
        b.gameobject = Instantiate(bt.prefab, SLAPUtils.LatLonToUnityCoords(lat_long), Quaternion.identity, gameObject.transform);
        
        // add to list of current beacons
        curr_beacons.Add(b);
        var mission_object = b.gameobject.GetComponent<SLAPMissionObject>();
        //.latlon = b.lat_long;
        //map_manager.AddIcon(bt, b);
        */
    }
    #endregion
}
