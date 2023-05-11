using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class SLAPAstronautHandler : MonoBehaviour
{
    #region fields
    public static SLAPAstronautHandler instance;

    public string astronaut_name = "ARSIS4";
    /*[Range(1, 23)]
    public int room_num = 1;*/

    // rate at which getastronautlatlon is called -- if initialization
    // happens twice for whatever reason this will break
    [Range(3, 60)]
    public int latlon_refresh_rate;

    [Header("debug values, don't change")]
    public int tracked_user = 0;
    public UserLocation user;
    public SLAPUtils.LatLon latlon = new SLAPUtils.LatLon(0, 0);
    private APICallHandler ach;
    #endregion

    void Awake ()
    {
        instance = this;
    }

    public async Task<SLAPUtils.LatLon> Initialize ()
    {
        ach = APICallHandler.instance;
        ach.CreateUser(astronaut_name, ach.requestFields["room"]);

        //var retval =  await GetAstronautLatLon();
        var retval = await GetAstronautLatLon();
        InvokeRepeating("GetAstronautLatLon", 2.0f, latlon_refresh_rate);

        return latlon;
    }

    public async Task<SLAPUtils.LatLon> GetAstronautLatLon ()
    {
        await ach.GetLocationDataByRoom(ach.requestFields["room"]);

        // find the most recently-registered user
        tracked_user = 0;
        foreach (var u in ach.locationData.users)
        {
            int tmp = int.Parse(u.user);
            if (tmp > tracked_user)
            {
                tracked_user = tmp;
                user = u;
            }
        }

        latlon.lat = user.latitude;
        latlon.lon = user.longitude;

        //return retval;
        return latlon;
    }
}
