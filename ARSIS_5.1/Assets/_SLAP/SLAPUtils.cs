

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Threading.Tasks;

public class SLAPUtils : MonoBehaviour
{
    public static SLAPAstronautHandler astronaut;

    public static LatLon latlon_at_origin_task_input = new LatLon(0, 0);
    public static LatLon latlon_at_origin = new LatLon(0,0);

    public static LatLon latlon_at_second_point_task_input = new LatLon(0,0);
    public static LatLon latlon_at_second_point = new LatLon(0, 0);

    public static Vector3 v3_at_second_point = new Vector3(0, 0, 10);

    public static float theta = 0;

    [Serializable]
    public class LatLon
    {
        public double lat;
        public double lon;

        public LatLon(double lat, double lon)
        {
            this.lat = lat;
            this.lon = lon;
        }

        public string toString ()
        {
            return "{lat: " + lat + "long: " + lon + " }";
        }
    }

    #region callibration stuff
    // I put this in it's own region so that you don't have to look at it
    // please just carry on
    public static async Task<bool> CallibrateStartPosAsync()
    {
        latlon_at_origin_task_input = await astronaut.Initialize();
        latlon_at_origin.lat = latlon_at_origin_task_input.lat;
        latlon_at_origin.lon = latlon_at_origin_task_input.lon; // i hate this so much, please, have mercy on my soul
        Debug.Log("latlon at origin -- lat: " + latlon_at_origin.lat 
            + " lon: " + latlon_at_origin.lon);
        return true;
    }

    public static async Task<bool> CallibrateSecondPosAsync (Vector3 unity_pos)
    {
        latlon_at_second_point_task_input = await astronaut.GetAstronautLatLon();
        latlon_at_second_point.lat = latlon_at_second_point_task_input.lat;
        latlon_at_second_point.lon = latlon_at_second_point_task_input.lon; // i hate this so much, please, have mercy on my soul copypasete
        v3_at_second_point = unity_pos;
        Debug.Log("latlon at second point -- lat: " + latlon_at_second_point.lat
            + " lon: " + latlon_at_second_point.lon);

        BeaconsManager.instance.SpawnInitialBeacons();
        return true;
    }
    #endregion

    /*
     * this is arcane to me, but was constructed due to the valiant efforts of
     * dr. swanson and megan
     */
    public static Vector3 LatLonToUnityCoords(LatLon latlon)
    {
        //Only calculated for initial calibration
        double x = ((latlon_at_origin.lon - latlon_at_second_point.lon) * Math.Cos(latlon_at_second_point.lat) * 111321.5);
        double y = ((latlon_at_second_point.lat - latlon_at_origin.lat) * 111321.5);
        float theta = (float)(Math.Atan2(x, y));

        //Calculated for every new coordinate
        double N = ((latlon.lat - latlon_at_origin.lat) * 111321.5); 
        double E = ((latlon_at_origin.lon - latlon.lon) * Math.Cos(latlon_at_second_point.lat) * 111321.5);

        double Z = ((Math.Cos(theta) * N) + (Math.Sin(theta) * E));
        double X = ((-Math.Sin(theta) * N) + (Math.Cos(theta) * E));

        Debug.Log("calculated unity coords for lat/lon! " + new Vector3((float)X, 0, (float)Z));
        return new Vector3((float)X, 0, (float)Z);
        
    }

    #region experimental triangulation stuff -- not currently in use
    /*
    public static Vector3 TriangulateUnityCoords (LatLon target, ReferencePoint[] references)
    {
        return Vector3.zero;
    }

    // should return meters, sourced from here:
    // https://stackoverflow.com/questions/6366408/calculating-distance-between-two-latitude-and-longitude-geocoordinates
    public double GetDistance(double longitude, double latitude, double otherLongitude, double otherLatitude)
    {
        var d1 = latitude * (Math.PI / 180.0);
        var num1 = longitude * (Math.PI / 180.0);
        var d2 = otherLatitude * (Math.PI / 180.0);
        var num2 = otherLongitude * (Math.PI / 180.0) - num1;
        var d3 = Math.Pow(Math.Sin((d2 - d1) / 2.0), 2.0) + Math.Cos(d1) * Math.Cos(d2) * Math.Pow(Math.Sin(num2 / 2.0), 2.0);

        var retval = 6376500.0 * (2.0 * Math.Atan2(Math.Sqrt(d3), Math.Sqrt(1.0 - d3)));
        Debug.Log("distance calculated! " + retval);
        return retval;
    }



    public static Vector2 LatLonToMapCoords(LatLon latlon)
    {
        return Vector2.zero;
    }*/
    #endregion
}

