using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using EventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;

public class TelemetryClient : MonoBehaviour
{
    //TODO We need to create a user that can be set by either env vars or get its user id from the server as it connects as a UUID
    private static string userMockId = "user1";
    private static string telemetryServerUrl = "http://localhost:8080";
    private static string telemetryServerLocation = telemetryServerUrl + "/location/" + userMockId;
    private static string telemetryServerBiometrics = telemetryServerUrl + "/biometrics/" + userMockId;
    private static string telemetryServerBiometricsBPM = telemetryServerUrl + "/biometrics/" + userMockId + "/bpm";
    private static string telemetryServerBiometricsO2 = telemetryServerUrl + "/biometrics/" + userMockId + "/o2";
    private static string telemetryServerBiometricsBattery = telemetryServerUrl + "/biometrics/" + userMockId + "/battery";
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
            /* StartCoroutine(GetBiometrics()); */
            StartCoroutine(StartPollingEndpoint<BiometricsEvent>(telemetryServerBiometrics, telemetryPollingDelay));
            StartCoroutine(StartPollingEndpoint<LocationEvent>(telemetryServerLocation, telemetryPollingDelay));
            yield return null;
    }

    IEnumerator StartPollingEndpoint<Event>(string endpoint, WaitForSeconds telemetryPollingDelay){
        while(true){
            UnityWebRequest www = UnityWebRequest.Get(endpoint);
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success) {
                Debug.Log(www.error);
            }
            else {
                string resultString = www.downloadHandler.text;
                Event newEvent = JsonConvert.DeserializeObject<Event>(resultString);
                EventManager.Trigger(newEvent);
            }
            yield return telemetryPollingDelay;
        }
    }
}
