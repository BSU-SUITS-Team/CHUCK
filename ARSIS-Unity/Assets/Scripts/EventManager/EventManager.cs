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
        public WebSocketClient Client { get; private set; }
        public string Endpoint { get; set; } = "ws://localhost:8181/ws/events";
        private IEnumerator coroutine;
        private EventDatastore eventDatastore = EventDatastore.Instance;

        [ContextMenu("Start Client")]
        public void StartClient()
        {
            if (coroutine != null)
            {
                Debug.Log("Client coroutine already started!");
                return;
            }
            Client = new WebSocketClient(Endpoint);
            coroutine = Client.StartClient();
            StartCoroutine(coroutine);
        }

        [ContextMenu("End Client")]
        public void EndClient()
        {
            if (coroutine == null) return;
            Debug.Log("Stopping client coroutine!");
            StopCoroutine(coroutine);
            coroutine = null;
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
            if (Client != null && eventDatastore != null)
            {
                BaseArsisEvent wsEvent = Client.GetEvent();
                if (wsEvent == null) return;
                if (wsEvent.label != null && wsEvent.label.Length > 0)
                {
                    eventDatastore.Upsert(wsEvent.type, wsEvent);
                }
                else
                {
                    eventDatastore.Append(wsEvent.type, wsEvent);
                }
            }
        }
    }
}
