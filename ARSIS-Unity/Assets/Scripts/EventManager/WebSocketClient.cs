using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class WebSocketClient
{
    private Uri endpoint { get; set; }
    private ClientWebSocket connection;
    private CancellationToken token = CancellationToken.None;

    public WebSocketClient(string endpoint) {
        this.endpoint = new Uri(endpoint);
        connection = new ClientWebSocket();
    }

    public async Task StartClient() {
        using (connection)
        {
            Debug.Log("WebSocketClient: starting connection to " + endpoint.ToString());
            await connection.ConnectAsync(endpoint, token);
            int offset = 0;
            byte[] buffer = new byte[4096];
            int maxIterations = 10;
            int iteration = 0;
            while (connection.State == WebSocketState.Open)
            {
                while (true)
                {
                    ArraySegment<byte> received = new ArraySegment<byte>(buffer);
                    WebSocketReceiveResult receiveResult = await connection.ReceiveAsync(received, token);
                    Debug.Log(Encoding.UTF8.GetString(buffer, offset, receiveResult.Count));
                    offset += receiveResult.Count;
                    if (receiveResult.EndOfMessage) break;
                }
                Debug.Log("Complete Response:");
                Debug.Log(Encoding.UTF8.GetString(buffer, 0, offset));
            }
        }
    }
}
