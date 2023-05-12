using System.Collections;
using System.Collections.Generic;
using SystemTasks = System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Networking;

public class APICallHandler : MonoBehaviour
{
    // expected endpoint does not contain /api or trailing /
    // valid examples: 127.0.0.1, localhost:8080, bsusuits2022.herokuapp.com

    #region unity logic
    public static APICallHandler instance;
    public string input;

    void Awake ()
    {
        instance = this;    // make singleton
    }
    #endregion

    #region fields
    [SerializeField]
    private string endpoint = "localhost:8080";

    [SerializeField]
    public SimulationData simulationState;

    [SerializeField]
    public SimulationControls simulationControl;

    [SerializeField]
    public LocationData locationData;

    [SerializeField]
    public LSARData lsarData;

    [UDictionary.Split(30, 70)]
    public BodyFields requestFields;
    [System.Serializable]
    public class BodyFields : UDictionary<string, string> { }

    [SerializeField]
    public WWWForm body;

    #endregion

    #region utility functions
    public void EndpointInput(string s)
    {
        input = s;
        s = endpoint;
    }

    public void SetEndpoint(string endpoint)
    {
        this.endpoint = endpoint;
    }

    public string GetEndpoint()
    {
        return this.endpoint;
    }

    private bool IsValidEndpoint()
    {
        if (string.IsNullOrEmpty(endpoint))
        {
            Debug.LogError("Endpoint is null or empty!");
            return false;
        }

        return true;
    }

    public async SystemTasks.Task<string> SendGetRequest(string route)
    {
        string response;

        using (UnityWebRequest request = UnityWebRequest.Get(endpoint + route))
        {
            UnityWebRequestAsyncOperation operation = request.SendWebRequest();

            while (!operation.isDone)
            {
                await SystemTasks.Task.Yield();
            }

            if (request.result == UnityWebRequest.Result.ProtocolError)
            {
                response = request.error;
                Debug.Log($"Error: {response}");
            } else {
                response = request.downloadHandler.text;
                Debug.Log($"Received: {response}");
            }
        }

        return response;
    }

    public async SystemTasks.Task<string> SendPostRequest(string route, WWWForm body)
    {
        string response;

        using (UnityWebRequest request = UnityWebRequest.Post(endpoint + route, body))
        {
            UnityWebRequestAsyncOperation operation = request.SendWebRequest();

            while (!operation.isDone)
            {
                await SystemTasks.Task.Yield();
            }

            if (request.result == UnityWebRequest.Result.ProtocolError)
            {
                response = request.error;
                Debug.Log($"Error: {response}");
            } else {
                response = request.downloadHandler.text;
                Debug.Log($"Received: {response}");
            }
        }

        return response;
    }

    public async SystemTasks.Task<string> SentPutRequest(string route, WWWForm body)
    {
        string response;

        using (UnityWebRequest request = UnityWebRequest.Put(endpoint + route, body.data))
        {
            UnityWebRequestAsyncOperation operation = request.SendWebRequest();

            while (!operation.isDone)
            {
                await SystemTasks.Task.Yield();
            }

            if (request.result == UnityWebRequest.Result.ProtocolError) 
            {
                response = request.error;
                Debug.Log($"Error: {response}");
            } else {
                response = request.downloadHandler.text;
                Debug.Log($"Received: {response}");
            }
        }

        return response;
    }

    public async SystemTasks.Task<string> SentRemoveRequest(string route)
    {
        string response;

        using (UnityWebRequest request = UnityWebRequest.Delete(endpoint + route))
        {
            UnityWebRequestAsyncOperation operation = request.SendWebRequest();

            while (!operation.isDone)
            {
                await SystemTasks.Task.Yield();
            }

            if (request.result == UnityWebRequest.Result.ProtocolError) 
            {
                response = request.error;
                Debug.Log($"Error: {response}");
            } else {
                response = request.downloadHandler.text;
                Debug.Log($"Received: {response}");
            }
        }

        return response;
    }

    #endregion

    #region other functions

    #region verification

    public void GetAPI()
    {
        if (IsValidEndpoint())
        {
            SendGetRequest("");
        }
    }

    public void GetRooms()
    {
        string route = "/api/rooms";

        if (IsValidEndpoint())
        {
            SendGetRequest(route);
        }
    }

    public void GetUsersInRoom(int roomNumber)
    {
        string route = $"/api/users/room/{roomNumber}";

        if (IsValidEndpoint())
        {
            SendGetRequest(route);
        }
    }

    #endregion

    #region simulation state

    public async void GetSimulationStateInRoom(int room)
    {
        string route = $"/api/simulationstate/{room}";

        if (IsValidEndpoint())
        {
            string jsonData = await SendGetRequest(route);
            this.simulationState = JsonUtility.FromJson<SimulationData>(jsonData);
        }
    }

    public async void GetSimulationStateInRoom(string room)
    {
        string route = $"/api/simulationstate/{room}";

        if (IsValidEndpoint())
        {
            string jsonData = await SendGetRequest(route);
            this.simulationState = JsonUtility.FromJson<SimulationData>(jsonData);
        }
    }

    public async void SendSimulationControls(int room, string control)
    {
        string route = $"/api/simulationcontrol/simctl/{room}/{control}";

        if (IsValidEndpoint())
        {
            string jsonData = await SendGetRequest(route);
            this.simulationControl = JsonUtility.FromJson<SimulationControls>(jsonData);
        }
    }

    public async void SendSimulationFailure(int room, string failure)
    {
        string route = $"/api/simulationcontrol/simfail/{room}/{failure}";

        if (IsValidEndpoint())
        {
            string jsonData = await SendGetRequest(route);
            this.simulationControl = JsonUtility.FromJson<SimulationControls>(jsonData);
        }
    }

    #endregion

    #region user stuff

    public void CreateUser(string username)
    {
        string route = "/api/auth/register";

        WWWForm body = new WWWForm();
        body.AddField("username", username);

        if (IsValidEndpoint())
        {
            SendPostRequest(route, body);
        }
    }

    public void CreateUser(string username, string room)
    {
        string route = "/api/auth/register";

        WWWForm body = new WWWForm();
        body.AddField("username", username);
        body.AddField("room", room);

        if (IsValidEndpoint())
        {
            SendPostRequest(route, body);
        }
    }

    public void CreateUser(string username, int room)
    {
        string route = "/api/users/";

        WWWForm body = new WWWForm();
        body.AddField("username", username);
        body.AddField("room", room);

        if (IsValidEndpoint())
        {
            SendPostRequest(route, body);
        }
    }

    public void RegisterUserRoom(int room, string username)
    {
        string route = "/api/auth/register/";

        WWWForm body = new WWWForm();
        body.AddField("username", username);
        body.AddField("room", room);

        if (IsValidEndpoint())
        {
            SendPostRequest(route, body);
        }
    }

    public void RegisterUserRoom(int room, WWWForm body)
    {
        string route = "/api/auth/register/";

        if (IsValidEndpoint())
        {
            SendPostRequest(route, body);
        }
    }

    #endregion

    #region location data

    public async SystemTasks.Task<LocationData> GetLocationDataById(int id)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/locations/user/{id}";
        string jsonData = await SendGetRequest(route);
        LocationData received = JsonUtility.FromJson<LocationData>($"{{\"users\":{jsonData}}}");
        locationData = received;
        return received;
    }

    public async SystemTasks.Task<LocationData> GetLocationDataByRoom(int room)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/locations/room/{room}";
        string jsonData = await SendGetRequest(route);
        LocationData received = JsonUtility.FromJson<LocationData>($"{{\"users\":{jsonData}}}");
        locationData = received;
        return received;
    }

    public async SystemTasks.Task<LocationData> GetLocationDataById(string id)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/locations/user/{id}";
        string jsonData = await SendGetRequest(route);
        LocationData received = JsonUtility.FromJson<LocationData>($"{{\"users\":{jsonData}}}");
        locationData = received;
        return received;
    }

    public async SystemTasks.Task<LocationData> GetLocationDataByRoom(string room)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/locations/room/{room}";
        string jsonData = await SendGetRequest(route);
        LocationData received = JsonUtility.FromJson<LocationData>($"{{\"users\":{jsonData}}}");
        locationData = received;
        return received;
    }

    #endregion

    #region LSAR data

    public async SystemTasks.Task<LSARMessage> GetLSARDataById(int id)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/lsar/{id}";
        string jsonData = await SendGetRequest(route);
        LSARMessage received = JsonUtility.FromJson<LSARMessage>(jsonData);
        lsarData.messages = new LSARMessage[] {received};
        return received;
    }

    public async SystemTasks.Task<LSARData> GetLSARDataByRoom(int room)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/lsar/room/{room}";
        string jsonData = await SendGetRequest(route);
        LSARData received = JsonUtility.FromJson<LSARData>($"{{\"messages\":{jsonData}}}");
        lsarData = received;
        return received;
    }

    public async SystemTasks.Task<LSARMessage> GetLSARDataById(string id)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/lsar/{id}";
        string jsonData = await SendGetRequest(route);
        LSARMessage received = JsonUtility.FromJson<LSARMessage>(jsonData);
        lsarData.messages = new LSARMessage[] { received };
        return received;
    }

    public async SystemTasks.Task<LSARData> GetLSARDataByRoom(string room)
    {
        if (!IsValidEndpoint()) return null;

        string route = $"/api/lsar/room/{room}";
        string jsonData = await SendGetRequest(route);
        LSARData received = JsonUtility.FromJson<LSARData>($"{{\"messages\":{jsonData}}}");
        lsarData = received;
        return received;
    }

    #endregion

    #endregion
}