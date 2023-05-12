using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class LSARData : ISerializationCallbackReceiver
{
    public LSARMessage[] messages;

    public void OnBeforeSerialize() { }
    public void OnAfterDeserialize() { }
}

[System.Serializable]
public class LSARMessage : ISerializationCallbackReceiver
{
    public static string OK = "OK";
    public static string ON_MY_WAY = "ONMYWAY";
    public static string COPY = "COPY";
    public static string GOOD_COMM = "GOODCOMM";
    public static string NEGATIVE = "NEGATIVE";
    public static string AFFRIM = "AFFIRM";
    public static string HIGH = "HIGH";
    public static string NORMAL = "NORMAL";
    public static string SURFACE = "SURFACE";
    public static string SPACE = "SPACE";
    public static string HOMING = "HOMING";
    public static string NON_HOMING = "NON-HOMING";
    public static string YES = "YES";
    public static string NO = "NO";
    public static string TEST = "TEST";
    public static string NON_TEST = "NON-TEST";
    public static string UNPRESSURIZED_CREW_ROVER = "000";
    public static string PRESSURIZED_CREW_ROVER = "001";
    public static string EVA_CREW_MEMBER = "010";
    public static string HLS_LANDER = "011";
    public static string FIXED_LOCATION = "111";

    public int id;
    public int sender;
    public int room;
    public System.DateTime Time;
    public string priority_tag;
    public double encoded_lat;
    public double encoded_lon;
    public string pnt_source;
    public string condition_state;
    public string vmc_txt;
    public string tac_sn;
    public string cntry_code;
    public string homing_dvc_stat;
    public string ret_lnk_stat;
    public string test_proto;
    public string vessel_id;
    public string beacon_type;
    [SerializeField] private string time;

    public void OnBeforeSerialize()
    {
        time = Time.ToString("o");
    }

    public void OnAfterDeserialize()
    {
        System.DateTime.TryParse(time, out Time);
    }
}