using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;
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

public class RegisteredUsersDict {
    public List<RegisteredUser> users;
    public RegisteredUsersDict(List<RegisteredUser> users){
        this.users = users;
    }
}

public class TelemetryClient : MonoBehaviour
{
    public static string ip = "192.168.0.107";
    private static string userMockName = "user1";
    private static string telemetryServerUrl = "http://" + ip + ":8080";
    private static string telemetryServerUser = telemetryServerUrl + "/user";
    private string telemetryServerLocation = telemetryServerUrl + "/location/";
    private string telemetryServerBiometrics = telemetryServerUrl + "/biometrics/";

    enum Endpoint {LOCATION, BIOMETRICS, USER};
    private Dictionary<Endpoint, string> serverEndpointDict = new Dictionary<Endpoint, string>();

    private WaitForSeconds telemetryPollingDelay = new WaitForSeconds(1.0f);

    public RegisteredUser registeredUser;

    // Start is called before the first frame update
    void Start()
    {
        updateServerEndpointDict();
        StartCoroutine(PopulateRegisterdUser());
        StartCoroutine(StartPollingTelemetryApi());
    }

    void updateServerEndpointDict(){
        string userId = "";
        if (registeredUser != null){
            userId = registeredUser.id.ToString();
        }
        serverEndpointDict[Endpoint.USER] = telemetryServerUser;
        serverEndpointDict[Endpoint.LOCATION] = telemetryServerLocation + userId;
        serverEndpointDict[Endpoint.BIOMETRICS] = telemetryServerBiometrics + userId;

    }
    IEnumerator PopulateRegisterdUser(){
        StartCoroutine(CheckIfUsernameIsUsed());
        yield return null;
    }

    IEnumerator CheckIfUsernameIsUsed(){
        UnityWebRequest www = UnityWebRequest.Get(telemetryServerUser);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            string resultString = www.downloadHandler.text;
            RegisteredUsersDict registeredUsersDict = JsonConvert.DeserializeObject<RegisteredUsersDict>(resultString);
            foreach (RegisteredUser r in registeredUsersDict.users){
                if (r.name == userMockName){
                    registeredUser = r;
                    updateServerEndpointDict();
                }
            }
        }
        StartCoroutine(RegisterWithApi());
        yield return null;
    }
    IEnumerator RegisterWithApi(){
        while(registeredUser == null){
            UserToRegister user = new UserToRegister(userMockName);
            string userData = JsonConvert.SerializeObject(user);
            byte[] myData = System.Text.Encoding.UTF8.GetBytes(userData);
            using (UnityWebRequest www = UnityWebRequest.Put(telemetryServerUser, myData))
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
                    registeredUser = JsonConvert.DeserializeObject<RegisteredUser>(resultString);
                    updateServerEndpointDict();
                }
            }
            yield return telemetryPollingDelay;
        }
    }

    IEnumerator StartPollingTelemetryApi() {
        while(registeredUser == null){
            yield return telemetryPollingDelay;
        }
            /* StartCoroutine(StartPollingEndpoint<BiometricsEvent>(telemetryServerBiometrics, telemetryPollingDelay)); */
            StartCoroutine(StartPollingEndpoint<LocationEvent>(Endpoint.LOCATION, telemetryPollingDelay));
            yield return null;
    }

    IEnumerator StartPollingEndpoint<Event>(Endpoint endpoint, WaitForSeconds telemetryPollingDelay){
        while(true){
            yield return telemetryPollingDelay;
            if (registeredUser == null){
                continue;
            }
            string endpointUrl = serverEndpointDict[endpoint];
            UnityWebRequest www = UnityWebRequest.Get(endpointUrl);
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success) {
                Debug.Log(www.error);
            }
            else {
                string resultString = www.downloadHandler.text;
                Event newEvent = JsonConvert.DeserializeObject<Event>(resultString);
                EventManager.Trigger(newEvent);
            }
        }
    }
}
