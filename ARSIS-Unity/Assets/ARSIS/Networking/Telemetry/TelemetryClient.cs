using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using EventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;

public class TelemetryClient : MonoBehaviour
{
    private static string telemetryServerUrl = "http://localhost:8080";
    private static string telemetryServerLocation = telemetryServerUrl + "/location/";
    private static string telemetryServerBiometrics = telemetryServerUrl + "/biometrics/";
    private WaitForSeconds telemetryPollingDelay = new WaitForSeconds(1.0f);
    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(StartPollingTelemetryApi());
    }

    // Update is called once per frame
    void Update()
    {

    }
    IEnumerator StartPollingTelemetryApi() {
        while(true){
            StartCoroutine(GetBiometrics());
            StartCoroutine(GetLocation());
            yield return telemetryPollingDelay;
        }
    }
    IEnumerator GetBiometrics() {
        UnityWebRequest www = UnityWebRequest.Get(telemetryServerBiometrics);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            // Show results as text
            /* Debug.Log(www.downloadHandler.text); */

        }
    }
    IEnumerator GetLocation() {
        UnityWebRequest www = UnityWebRequest.Get(telemetryServerLocation);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            string resultString = www.downloadHandler.text;
            LocationEvent newLocationEvent = JsonConvert.DeserializeObject<LocationEvent>(resultString);
            EventManager.Trigger(newLocationEvent);
        }
    }
}
