using Newtonsoft.Json;
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
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
        private const long NanosecondsPerTick = 100L;
        private static readonly DateTime UnixEpoch = new(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        private static readonly HashSet<long> SeenCommandTimes = new();
        private static readonly object SeenCommandTimesLock = new();
        private readonly long applicationStartTimeNs;

        public WebSocketClient(string endpoint) : this(endpoint, GetUnixTimeNanoseconds()) { }

        public WebSocketClient(string endpoint, long applicationStartTimeNs) {
            this.endpoint = endpoint;
            this.applicationStartTimeNs = applicationStartTimeNs;
        }

        public static long GetUnixTimeNanoseconds()
        {
            return (DateTime.UtcNow.Ticks - UnixEpoch.Ticks) * NanosecondsPerTick;
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
            if (wsEvent == null) return;

            if (ShouldDiscardCommand(wsEvent))
                return;

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

        private bool ShouldDiscardCommand(BaseArsisEvent wsEvent)
        {
            if (!(wsEvent is HololensCommand) && !(wsEvent is ArmbarButtonPress))
                return false;

            if (wsEvent.time <= applicationStartTimeNs)
            {
                Debug.Log($"Discarding stale {wsEvent.type} event from before app start.");
                return true;
            }

            lock (SeenCommandTimesLock)
            {
                if (SeenCommandTimes.Contains(wsEvent.time))
                {
                    Debug.Log($"Discarding duplicate {wsEvent.type} event with time {wsEvent.time}.");
                    return true;
                }

                SeenCommandTimes.Add(wsEvent.time);
            }

            return false;
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
