using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

public class UIAController : MonoBehaviour
{
    public UIAData uiaData; 

    public bool testing; 

    public string url;
    public string testUrl; 

public static UIAController S; 

    void Start()
    {
        S = this; 

        InvokeRepeating("GetUIAData", 1, 1);
    }

    public string checkVar(string varName)
    {
        switch (varName) {
            case "O2_vent":
                return uiaData.O2_vent;
            case "depress_pump":
                return uiaData.depress_pump;
            case "emu1":
                return uiaData.emu1;
            case "emu1_O2":
                return uiaData.emu1_O2;
            case "emu2":
                return uiaData.emu2;
            case "emu2_O2":
                return uiaData.emu2_O2;
            case "ev1_supply":
                return uiaData.ev1_supply;
            case "ev1_waste":
                return uiaData.ev1_waste;
            case "ev2_supply":
                return uiaData.ev2_supply;
            case "ev2_waste":
                return uiaData.ev2_waste;
            case "o2_supply_pressure1":
                return uiaData.o2_supply_pressure1.ToString();
            case "o2_supply_pressure2":
                return uiaData.o2_supply_pressure2.ToString();
            case "oxygen_supp_out1":
                return uiaData.oxygen_supp_out1.ToString();
            case "oxygen_supp_out2":
                return uiaData.oxygen_supp_out2.ToString();
            case "started_at":
                return uiaData.started_at;
            default:
                return ""; 

        }
    }

    private void GetUIAData()
    {
        StartCoroutine(RunWWW());
    }

    IEnumerator RunWWW()
    {
        if (!JSONParse.S.parsingOn) yield break;
        if (MenuController.s.currentProcedure != 0) yield break; // only get this data while on the procedure we need it for

        using (UnityWebRequest www = UnityWebRequest.Get(testing ? testUrl : url))
        {
            yield return www.SendWebRequest();

            string json = "";
            if (www.isNetworkError)
            {
                Debug.LogError("NETWORK ERROR Not connected to UIA server :(");
            }
            else if (www.isHttpError)
            {
                Debug.LogError("HTTP ERROR Not connected to UIA server :( :( :(");
            }
            else
            {
                // We are connected to the server 

                // Use line below only if the JSON comes in with brackets around it 
                //json = RemoveBrackets(www.downloadHandler.text);
                json = www.downloadHandler.text;

                //Debug.Log("Connected to UIA server");
            }

            if (!json.Equals(""))
            {
                //Debug.Log(www.downloadHandler.text);
                uiaData = JsonUtility.FromJson<UIAData>(www.downloadHandler.text);
            }
            else
            {
                Debug.Log("no data recieved from the server");
            }


        }
    }
}

[System.Serializable]
public class UIAData
{
    /* public string O2_vent = "";
     public string depress_pump = "";
     public string emu1 = "";
     public string emu1_O2 = "";
     public string emu2 = "";
     public string emu2_O2 = "";
     public string ev1_supply = "";
     public string ev1_waste = "";
     public string ev2_supply = "";
     public string ev2_waste = "";
     public string o2_supply_pressure1 = "";
     public string o2_supply_pressure2 = "";
     public string oxygen_supp_out1 = "";
     public string oxygen_supp_out2 = "";
     public string started_at = "";*/

    public string started_at = "";
    public string emu1 = "";
    public string emu2 = "";
    public int o2_supply_pressure1 = 0;
    public int o2_supply_pressure2 = 0;
    public string ev1_supply = "";
    public string ev2_supply = "";
    public string ev1_waste = "";
    public string ev2_waste = "";
    public string emu1_O2 = "";
    public string emu2_O2 = "";
    public int oxygen_supp_out1 = 0;
    public int oxygen_supp_out2 = 0;
    public string O2_vent = "";
    public string depress_pump = "";
}

/*{ "_id":"60764c84a44f8a7084110a5d",
        "started_at":"2021-04-14T01:59:32.302Z",
        "emu1":"OFF","emu2":"OFF",
        "o2_supply_pressure1":29,
        "o2_supply_pressure2":29,
        "ev1_supply":"CLOSE",
        "ev2_supply":"CLOSE",
        "ev1_waste":"CLOSE",
        "ev2_waste":"CLOSE",
        "emu1_O2":"CLOSE",
        "emu2_O2":"CLOSE",
        "oxygen_supp_out1":29,
        "oxygen_supp_out2":29,
        "O2_vent":"CLOSE",
        "depress_pump":"FAULT","__v":0}*/