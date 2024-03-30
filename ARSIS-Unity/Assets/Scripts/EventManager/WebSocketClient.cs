using System;
using System.Collections;
using System.Collections.Concurrent;
using UnityEngine;
using WebSocketSharp;
using WebSocket = WebSocketSharp.WebSocket;

namespace ARSIS.EventManager
{
    public class WebSocketClient
    {
        private string endpoint { get; set; } = "ws://localhost:8181/ws/events";
        private ConcurrentQueue<string> sendQueue = new();
        private ConcurrentQueue<string> receiveQueue = new();
        private WebSocket connection;
        private readonly int delay = 1; // number of seconds to wait before reconnecting

        public WebSocketClient(string endpoint) {
            this.endpoint = endpoint;
        }

        /// <summary>
        /// Performs reflection on the JSON payload and returns an object with superclass BaseArsisEvent.
        /// </summary>
        /// <param name="json"></param>
        /// <returns>BaseArsisEvent or null if failed to parse JSON.</returns>
        private BaseArsisEvent PerformReflection(string json)
        {
            BaseArsisEvent unknownEvent = JsonUtility.FromJson<BaseArsisEvent>(json);
            if (unknownEvent == null) return null;
            Type eventType = BaseArsisEvent.GetType(unknownEvent.type);
            return (BaseArsisEvent)JsonUtility.FromJson(json, eventType);
        }

        private IEnumerator AttemptReconnect(CloseEventArgs e)
        {
            if (!e.WasClean && !connection.IsAlive)
            {
                Debug.Log("Attempting to reconnect...");
                connection.ConnectAsync();
                yield return new WaitForSeconds(delay);
            }
        }

        private void Collect(MessageEventArgs e)
        {
            EventDatastore eventDatastore = EventDatastore.Instance;
            BaseArsisEvent wsEvent = PerformReflection(e.Data.ToString());
            Debug.Log(wsEvent);
            if (wsEvent.label != null && wsEvent.label.Length > 0)
            {
                eventDatastore.Upsert(wsEvent.type, wsEvent);
            }
            else
            {
                eventDatastore.Append(wsEvent.type, wsEvent);
            }
        }

        /// <summary>
        /// Establishes the WebSocket connection and begins listening to the events.
        /// </summary>
        /// <returns></returns>
        public void StartClient()
        {
            connection ??= new WebSocket(endpoint);
            connection.OnOpen += (sender, e) => Debug.Log("WebSocket connected!");
            connection.OnMessage += (sender, e) => Collect(e);
            connection.OnError += (sender, e) => EndClient();
            connection.OnClose += (sender, e) => AttemptReconnect(e);
            connection.ConnectAsync();
        }

        public void EndClient()
        {
            Debug.Log("Closing connection...");
            connection.CloseAsync();
        }
    }
}
