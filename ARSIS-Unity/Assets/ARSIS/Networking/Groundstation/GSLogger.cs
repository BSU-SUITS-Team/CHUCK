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
        /* StartCoroutine(GSLoggerCoroutine(loggingString)); */
        StartCoroutine(GSLoggerCoroutine(loggingBytes));
    }

    /* IEnumerator GSLoggerCoroutine(string loggingString){ */
    IEnumerator GSLoggerCoroutine(byte[] loggingBytes){
        Debug.Log(loggingBytes);
        WWWForm form = new WWWForm();
        form.AddField("data", "test");

        /* List<IMultipartFormSection> formData = new List<IMultipartFormSection>(); */
        /* formData.Add(new MultipartFormDataSection("data="+loggingString)); */
        Debug.Log(form.ToString());
        byte[] myData = System.Text.Encoding.UTF8.GetBytes("{\"data\": \"User01\"}");
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
