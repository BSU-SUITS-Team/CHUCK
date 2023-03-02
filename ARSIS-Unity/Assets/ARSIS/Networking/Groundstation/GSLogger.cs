using System.Collections;
using System.Collections.Generic;
using UnityEngine.Networking;
using UnityEngine;
using EventSystem;
using Newtonsoft.Json;

public class GSLogger : MonoBehaviour
{
  // Start is called before the first frame update
    private static string groundStationUrl = "http://localhost:8181/logger/";
    void Start()
    {
        EventManager.AddListenerToAll(GSLoggerCallback);
        // StartCoroutine(GetLog());
    }

    /* 
        Added Coroutines (GetLog) for quick testing without an event manager. feel free to remove.
  */
    IEnumerator GetLog() {
        int logId = 1;
        string groundStationUrlGetLog = groundStationUrl + logId.ToString();
        while (true)
            {
            UnityWebRequest www = UnityWebRequest.Get(groundStationUrlGetLog);
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success)
            {
                Debug.Log(www.error);
            }
            else
            {
                // Show results as text
                Debug.Log(www.downloadHandler.text);

            }
            yield return new WaitForSeconds(0.5f);
            }
    }

    void GSLoggerCallback(dynamic data)
    {
        byte[] loggingBytes = System.Text.Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(data));
        string loggingString = JsonConvert.SerializeObject(data);
        Debug.Log(loggingString);
        string escapedString = loggingString.Replace("\"", "\\\"");
        Debug.Log(escapedString);
        StartCoroutine(GSLoggerCoroutine(escapedString));
        /* StartCoroutine(GSLoggerCoroutine(loggingBytes)); */
    }

    IEnumerator GSLoggerCoroutine(string loggingString)
    {
        /* IEnumerator GSLoggerCoroutine(byte[] loggingBytes){ */

        string toSend = "{\"data\": \"" + loggingString + "\"}";
        Debug.Log(toSend);
        byte[] myData = System.Text.Encoding.UTF8.GetBytes(toSend);
        using (UnityWebRequest www = UnityWebRequest.Put(groundStationUrl, myData))
        {
        www.SetRequestHeader("accept", "application/json");
        www.SetRequestHeader("Content-Type", "application/json");
        yield return www.SendWebRequest();
        if (www.result != UnityWebRequest.Result.Success)
        {
            Debug.Log(www.error);
            foreach (var kvp in www.GetResponseHeaders())
            {
            Debug.Log("Key = " + kvp.Key + ", Value = " + kvp.Value);
            }
            Debug.Log(www.downloadHandler.text);
        }
        else
        {
            Debug.Log("Upload complete!");
        }
        }

    }
    }
