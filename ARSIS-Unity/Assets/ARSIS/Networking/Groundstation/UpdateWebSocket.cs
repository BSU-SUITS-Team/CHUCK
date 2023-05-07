using System.Collections.Generic;
using System;
using UnityEngine;
using WebSocketSharp;
using ARSISEventSystem;

public class UpdateWebSocket : MonoBehaviour
{
    private WebSocket ws;
    Dictionary<string, object> updateDict;

    private static string ws_update_url = "ws://127.0.0.1:8181/ws/updates";
    private void Start()
    {
        ws = new WebSocket(ws_update_url);
        ws.Connect();
        ws.OnMessage += (sender, e) =>
        {
            ParseWSData(e);
        };
        updateDict = new Dictionary<string, object>();

        updateDict.Add("NAVIGATION",new UpdateNavigationEvent());
        updateDict.Add("PROCEDURES",new UpdateProceduresEvent());
    }
    private void Update()
    {
        /* Debug.Log(ws.IsAlive); */
    }
    private void ParseWSData(WebSocketSharp.MessageEventArgs e)
    {
        Debug.Log(e.Data);
        object EventToTrigger = updateDict.GetValueOrDefault(e.Data);
        Debug.Log(EventToTrigger.GetType());
        EventManager.Trigger(EventToTrigger);
    }

}
