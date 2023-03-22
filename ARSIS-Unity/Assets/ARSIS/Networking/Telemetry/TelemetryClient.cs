using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using EventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;

public class UserToRegister {
    public string name;
    public UserToRegister(string name){
        this.name = name;
    }
}
public class RegisteredUser{
    public int id;
    public string name;
    public string createdAt;
    public RegisteredUser(int id, string name, string createdAt){
        this.id = id;
        this.name = name;
        this.createdAt = createdAt;
    }
}
public class TelemetryClient : MonoBehaviour
{
    private static string userMockName = "user1";
    private static string telemetryServerUrl = "http://localhost:8080";
    private static string registerUserUrl = telemetryServerUrl + "/user";
    private string telemetryServerLocation = telemetryServerUrl + "/location/";
    private string telemetryServerBiometrics = telemetryServerUrl + "/biometrics/";
    private WaitForSeconds telemetryPollingDelay = new WaitForSeconds(1.0f);

    public RegisteredUser registeredUser;

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(RegisterWithApi());
        StartCoroutine(StartPollingTelemetryApi());
    }

    // Update is called once per frame
    void Update()
    {

    }
    IEnumerator RegisterWithApi(){
        while(registeredUser == null){
            UserToRegister user = new UserToRegister(userMockName);
            string userData = JsonConvert.SerializeObject(user);
            byte[] myData = System.Text.Encoding.UTF8.GetBytes(userData);
            using (UnityWebRequest www = UnityWebRequest.Put(registerUserUrl, myData))
            {
                www.SetRequestHeader("accept", "application/json");
                www.SetRequestHeader("Content-Type", "application/json");
                yield return www.SendWebRequest();
                if (www.result != UnityWebRequest.Result.Success)
                {
                    Debug.Log(www.error);
                }
                else
                {
                    string resultString = www.downloadHandler.text;
                    Debug.Log(resultString);
                    registeredUser = JsonConvert.DeserializeObject<RegisteredUser>(resultString);

                }
            }
            telemetryServerLocation += registeredUser.id;
            telemetryServerBiometrics += registeredUser.id;
            yield return telemetryPollingDelay;
        }
    }

    IEnumerator StartPollingTelemetryApi() {
            /* StartCoroutine(StartPollingEndpoint<BiometricsEvent>(telemetryServerBiometrics, telemetryPollingDelay)); */
            StartCoroutine(StartPollingEndpoint<LocationEvent>(telemetryServerLocation, telemetryPollingDelay));
            yield return null;
    }

    IEnumerator StartPollingEndpoint<Event>(string endpoint, WaitForSeconds telemetryPollingDelay){
        while(true){
            yield return telemetryPollingDelay;
            if (registeredUser == null){
                /* Debug.Log(registeredUser); */
                continue;
            }
            UnityWebRequest www = UnityWebRequest.Get(endpoint);
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success) {
                Debug.Log(www.error);
            }
            else {
                string resultString = www.downloadHandler.text;
                Debug.Log(resultString);
                Event newEvent = JsonConvert.DeserializeObject<Event>(resultString);
                EventManager.Trigger(newEvent);
            }
        }
    }
}
