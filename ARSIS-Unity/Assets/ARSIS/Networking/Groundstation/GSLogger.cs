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
    }

    // Update is called once per frame
    void Update()
    {

    }

    void GSLoggerCallback(dynamic data){
        byte[] loggingBytes = System.Text.Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(data));
        string loggingString = JsonConvert.SerializeObject(data);
        StartCoroutine(GSLoggerCoroutine(loggingString));
    }

    IEnumerator GSLoggerCoroutine(string loggingString){
    /* IEnumerator GSLoggerCoroutine(byte[] loggingBytes){ */
        Debug.Log("incoro");
        List<IMultipartFormSection> formData = new List<IMultipartFormSection>();
        formData.Add(new MultipartFormDataSection("data="+loggingString));
        using (UnityWebRequest www = UnityWebRequest.Post(groundStationUrl, formData))
        {
            Debug.Log(www.url);
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success)
            {
                Debug.Log(www.error);
            }
            else
            {
                Debug.Log("Upload complete!");
            }
        }

    }
}
