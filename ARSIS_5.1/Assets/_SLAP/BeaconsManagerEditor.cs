#if UNITY_EDITOR

using UnityEngine;
using UnityEditor;
using System.Collections;

[CustomEditor(typeof(BeaconsManager))]
public class BeaconsManagerEditor : Editor
{
    //private Vector2 spawn_gps = Vector3.zero;
    private double spawn_lat;
    private double spawn_lon;

    override public void OnInspectorGUI()
    {
        BeaconsManager bm = (BeaconsManager)target;

        spawn_lat = EditorGUILayout.DoubleField("new beacon lat", spawn_lat);
        spawn_lon = EditorGUILayout.DoubleField("new beacon lon", spawn_lon);

        if (GUILayout.Button("Spawn example beacon"))
        {
            bm.SpawnBeacon(new SLAPUtils.LatLon(spawn_lat, spawn_lon), BeaconsManager.beacon_type_name.general, false);
        }

        // will break if unity isn't running
        if (GUILayout.Button("call calibration FROM THE SLAP MANAGER MENU"))
        {
            SLAPManager.instance.CalibratePosition();
        }

        DrawDefaultInspector();
    }
}

#endif