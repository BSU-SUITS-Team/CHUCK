using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class LunaSARManager : MonoBehaviour
{
    [Header("stuff for Menus")]
    public string outgoing_message = "ON MY WAY -- STAND BY";
    //private string sent_message;

    [Header("inspector stuffs")]
    public static LunaSARManager instance;
    [Range(3, 60)]
    public int lunasar_refresh_rate;
    public APICallHandler ach;
    public LSARMessage[] lsars;

    public AbbrevLunaSARResponse newest_recieved_lsar;

    [System.Serializable]
    public class AbbrevLunaSARResponse
    {
        public string message = "";
        public SLAPUtils.LatLon latlon;

        public AbbrevLunaSARResponse(string message, SLAPUtils.LatLon latlon)
        {
            this.message = message;
            this.latlon = latlon;
        }

        public bool Equals (AbbrevLunaSARResponse check)
        {
            // null check
            if (check == null) return false;

            // if not null
            bool retval = true;
            if (!message.Equals(check.message)) retval = false;
            if (!latlon.lat.Equals(check.latlon.lat)) retval = false;
            if (!latlon.lon.Equals(check.latlon.lon)) retval = false;
            return retval;
        }

        public string toString()
        {
            return message + " || lat: " + latlon.lat + ", lon: " + latlon.lon;
        }
    }
     
    void Awake ()
    {
        instance = this;
    }

    void Start ()
    {
        ach = APICallHandler.instance;
        // check for lunasar messages every x seconds
        InvokeRepeating("AskForLunaSAR", 2.0f, lunasar_refresh_rate);
    }

    // notify when a new lunasar message is recieved
    // display last recieved message (string, condition, lat/long)
    async void AskForLunaSAR ()
    {
        await ach.GetLSARDataByRoom (ach.requestFields["room"]);
        lsars = ach.lsarData.messages;
        
        if (lsars.Length > 0)
        {
            LSARMessage tmp = lsars[lsars.Length-1];

            // if the last element of the list was not sent by astronaut
            // display it
            // otherwise
            // check the one before.
            // defaults to last value
            for (int i = lsars.Length - 1; i >= 0; i--)
            {
                if (!lsars[i].beacon_type.Equals("999")) // 999 signifies a sent-from-local beacon
                {
                    tmp = lsars[i];
                    break;
                }
            }

            var tmp_2 = new AbbrevLunaSARResponse(
                tmp.condition_state,
                new SLAPUtils.LatLon(tmp.encoded_lat, tmp.encoded_lon)
                );

            if (!tmp_2.Equals(newest_recieved_lsar)) NewestLSARChanged();
            newest_recieved_lsar = tmp_2;

            //there might be a bug where if you send lsar from unity before you recieve one, it will treat the sent lunasar as a recieved lunasar
            //this is probably fine
        }
    }
    // format and send lunasar message
    public void RecievedMessage(string s)
    {
        outgoing_message = s;
    }
    public void SendLSAR ()
    {
        WWWForm body = new WWWForm();

        ach.requestFields["condition_state"] = outgoing_message;

        foreach (KeyValuePair<string, string> entry in ach.requestFields)
        {
            body.AddField(entry.Key, entry.Value);
        }

        ach.SendPostRequest("/api/lsar", body);
    }

    void NewestLSARChanged ()
    {
        Debug.Log("newest lsar changed");
        // spawn beacon and push to compass (automatic)
        BeaconsManager.instance.SpawnBeacon(newest_recieved_lsar.latlon, BeaconsManager.beacon_type_name.lunasar, true);
    }
}
