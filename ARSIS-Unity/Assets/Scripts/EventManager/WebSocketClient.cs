using Newtonsoft.Json;
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
            BaseArsisEvent unknownEvent = JsonConvert.DeserializeObject<BaseArsisEvent>(json);
            if (unknownEvent == null) return null;
            Type eventType = BaseArsisEvent.GetType(unknownEvent.type);
            return (BaseArsisEvent)JsonConvert.DeserializeObject(json, eventType);
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

        public string GetStatus()
        {
            if (connection == null) return "Not connected";
            switch (connection.ReadyState)
            {
                case WebSocketState.Open:
                    return "Open on " + endpoint;
                case WebSocketState.Closed:
                    return "Closed on " + endpoint;
                case WebSocketState.Connecting:
                    return "Connecting to " + endpoint;
                case WebSocketState.Closing:
                    return "Closing to " + endpoint;
                case WebSocketState.New:
                    return "New on " + endpoint;
                default:
                    return "Undefined status.";
            }
        }

        /// <summary>
        /// Establishes the WebSocket connection and begins listening to the events.
        /// </summary>
        /// <returns></returns>
        public void StartClient()
        {
            connection = new WebSocket(endpoint);
            connection.OnOpen += (sender, e) => Debug.Log("WebSocket connected!");
            connection.OnMessage += (sender, e) => Collect(e);
            connection.OnError += (sender, e) => {
                Debug.LogError(e.Exception.ToString());
                Debug.LogError(e.Message);
            };
            connection.OnClose += (sender, e) => AttemptReconnect(e);
            connection.ConnectAsync();
        }

        public void EndClient()
        {
            Debug.Log("Closing connection...");
            if (connection == null) return;
            connection.Close();
        }
    }
}
