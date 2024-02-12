using System;
using System.Collections;
using System.Collections.Concurrent;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace ARSIS.EventManager
{
    public class WebSocketClient
    {
        private Uri endpoint;
        private CancellationTokenSource cancellation = new();
        private ConcurrentQueue<string> sendQueue = new();
        private ConcurrentQueue<string> receiveQueue = new();

        public WebSocketClient(string endpoint) {
            this.endpoint = new Uri(endpoint);
        }

        /// <summary>
        /// Parses the string endpoint and provides a URI to the client.
        /// </summary>
        /// <param name="endpoint"></param>
        public void SetEndpoint(string endpoint)
        {
            this.endpoint = new Uri(endpoint);
        }

        /// <summary>
        /// Returns the endpoint.
        /// </summary>
        /// <returns>String of the endpoint.</returns>
        public string GetEndpoint()
        {
            return endpoint.ToString();
        }

        /// <summary>
        /// Gets an event from the receive queue of events from the WebSocket connection.
        /// </summary>
        /// <returns>BaseArsisEvent or null if queue is empty or failed to parse JSON.</returns>
        public BaseArsisEvent GetEvent()
        {
            try
            {
                if (receiveQueue.TryDequeue(out string json))
                    return PerformReflection(json);
                return null;
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                return null;
            }
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
            if (unknownEvent.type.Equals(Telemetry.Type))
                return JsonUtility.FromJson<Telemetry>(json);
            return unknownEvent;
        }

        /// <summary>
        /// Establishes the WebSocket connection and begins listening to the events.
        /// </summary>
        /// <returns></returns>
        public IEnumerator StartClient() {
            using (ClientWebSocket connection = new())
            {
                try
                {
                    Debug.Log("Connecting to WebSocket...");
                    connection.ConnectAsync(endpoint, cancellation.Token).Wait(cancellation.Token);
                }
                catch (Exception e)
                {
                    Debug.LogError(e);
                    throw;
                }
                byte[] buffer = new byte[2048];
                using (MemoryStream memoryStream = new())
                {
                    Debug.Log("Successfully connected to WebSocket!");
                    while (connection.State == WebSocketState.Open && !cancellation.IsCancellationRequested)
                    {
                        Task<WebSocketReceiveResult> task;
                        WebSocketReceiveResult result;
                        do
                        {
                            ArraySegment<byte> messageBuffer = WebSocket.CreateClientBuffer(2048, 16);
                            task = connection.ReceiveAsync(messageBuffer, cancellation.Token);
                            yield return new WaitUntil(() => task.IsCompleted);
                            if (task.IsFaulted) throw task.Exception;
                            result = task.Result;
                            memoryStream.Write(messageBuffer.Array, messageBuffer.Offset, result.Count);
                        }
                        while (!result.EndOfMessage);
                        switch (result.MessageType)
                        {
                            case WebSocketMessageType.Binary:
                                Debug.Log("Received binary: ");
                                break;
                            case WebSocketMessageType.Text:
                                string receivedMessage = Encoding.UTF8.GetString(memoryStream.ToArray());
                                Debug.Log("Received text: " + receivedMessage);
                                receiveQueue.Enqueue(receivedMessage);
                                memoryStream.Seek(0, SeekOrigin.Begin);
                                memoryStream.Position = 0;
                                memoryStream.SetLength(0);
                                break;
                            case WebSocketMessageType.Close:
                                yield break;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Requests the cancellation token to cancel the client from listening.
        /// </summary>
        public void EndClient()
        {
            cancellation.Cancel();
        }
    }
}
