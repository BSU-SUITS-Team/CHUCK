using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace ARSIS.EventManager
{
    [ExecuteInEditMode]
    public class EventManager : MonoBehaviour
    {

        public static EventManager Instance { get; private set; }
        public WebSocketClient client { get; private set; }
        public string endpoint { get; set; } = "ws://localhost:8181/ws/events";

        [ContextMenu("Start Client")]
        public void StartClient()
        {
            client = new WebSocketClient(endpoint);
            StartCoroutine(client.StartClient());
        }

        [ContextMenu("End Client")]
        public void EndClient()
        {
            client.EndClient();
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
            StartClient();
        }

        // Update is called once per frame
        void Update()
        {
            if (client != null)
            {
                BaseArsisEvent wsEvent = client.GetEvent();
                if (wsEvent != null)
                {
                    Debug.Log(wsEvent.ToString());
                }
            }
        }
    }
}
