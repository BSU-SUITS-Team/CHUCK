using System.Collections;
using System.Collections.Generic;
using UnityEngine.Networking;
using UnityEngine;
using EventSystem;
using Newtonsoft.Json;

public class GSLogger_COPYFORTEST : MonoBehaviour
{
    // Start is called before the first frame update
    private static string groundStationUrl = "http://localhost:8181/logger/";
    private static string groundStationUrlGetLog = "http://localhost:8181/logger/2"; // added for testing getlog by uuid endpoint
    void Start()
    {
        EventManager.AddListenerToAll(GSLoggerCallback);
        // StartCoroutine(GetLog());
        StartCoroutine(CreateLog());
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    /* 
        Added these Coroutines (GetLog and CreateLog) for quick testing without an event manager. feel free to remove.
    */
    IEnumerator GetLog()
    {
        while (true) {
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

    IEnumerator CreateLog() {
        while (true) {
            StartCoroutine(GSLoggerCoroutine("Hello World"));
            yield return new WaitForSeconds(0.5f);
        }
    }

    void GSLoggerCallback(dynamic data){
        byte[] loggingBytes = System.Text.Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(data));
        string loggingString = JsonConvert.SerializeObject(data);
        Debug.Log(loggingString);
        string escapedString = loggingString.Replace("\"","\\\"");
        Debug.Log(escapedString);
        StartCoroutine(GSLoggerCoroutine(escapedString));
        /* StartCoroutine(GSLoggerCoroutine(loggingBytes)); */
    }

    IEnumerator GSLoggerCoroutine(string loggingString){
    /* IEnumerator GSLoggerCoroutine(byte[] loggingBytes){ */

        string toSend = "{\"log\": \""+ loggingString +"\"}";
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
                foreach (var kvp in www.GetResponseHeaders()) {
                    Debug.Log("Key = "+ kvp.Key +", Value = "+ kvp.Value);
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
