using System.Collections.Generic;
using UnityEngine;
using WebSocketSharp;
using ARSISEventSystem;

public class UpdateWebSocket : MonoBehaviour
{
    private WebSocket ws;
    Dictionary<string, Type> updateDict;

    private static string ws_update_url = "ws://127.0.0.1:8181/ws/updates";
    private void Start()
    {
        ws = new WebSocket(ws_update_url);
        ws.Connect();
        ws.OnMessage += (sender, e) =>
        {
            ParseWSData(e);
        };

        updateDict.Add("NAVIGATION", ARSISEventSystem.UpdateNavigationEvent());
    }
    private void Update()
    {
        /* Debug.Log(ws.IsAlive); */
    }
    private void ParseWSData(WebSocketSharp.MessageEventArgs e)
    {
        /* SuitsEventType suitsEventType = (SuitsEventType)SuitsEventTelemetry.CreateFromJSON(e.Data); */
        Debug.Log(e.Data);
        /* Debug.Log("in wsclient " + suitsEventType.Type); */
        /* EventManager.TriggerEvent(suitsEventType); */
    }

}
