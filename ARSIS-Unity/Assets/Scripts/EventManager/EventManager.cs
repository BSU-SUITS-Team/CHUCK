using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

[ExecuteInEditMode]
public class EventManager : MonoBehaviour
{

    public static EventManager Instance { get; private set; }
    public WebSocketClient client { get; private set; }
    public string endpoint { get; set; } = "ws://localhost:8181/ws/events";

    [ContextMenu("Start Client")]
    public void StartClient()
    {
        WebSocketClient client = new WebSocketClient(endpoint);
        Task clientConnect = Task.Run(() => client.StartClient());
        clientConnect.Start();
    }

    void Awake() {
        if (Instance != null && Instance != this)
        {
            Destroy(this);
        }
        else
        {
            Instance = this;
        }
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
