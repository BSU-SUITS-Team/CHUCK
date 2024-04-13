using UnityEngine;

namespace ARSIS.EventManager
{
    public class EventManager : MonoBehaviour
    {

        public static EventManager Instance { get; private set; }
        public WebSocketClient Client { get; private set; }
        public string Endpoint { get; set; } = "ws://localhost:8181/ws/events";

        [ContextMenu("Start Client")]
        public void StartClient()
        {
            Client = new WebSocketClient(Endpoint);
            Client.StartClient();
        }

        [ContextMenu("End Client")]
        public void EndClient()
        {
            Client.EndClient();
        }

        void Awake()
        {
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
            StartClient();
        }

        void OnDestroy()
        {
            EndClient();
        }
    }
}
