using System.Collections;
using System.Collections.Generic;
using UnityEngine.Networking;
using UnityEngine;
using ARSIS.EventManager;

// This is mostrly for setting the remote URL used to talk to the MCP server
public class RemoteConfig : MonoBehaviour
{
    private static string URL = "https://config.suits.dev.plsignore.me/ip";
    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(GetIpAddress());
    }

    IEnumerator GetIpAddress()
    {
        UnityWebRequest request = UnityWebRequest.Get(URL);
        yield return request.SendWebRequest();

        if (request.result != UnityWebRequest.Result.Success)
        {
            Debug.Log(request.error);
        }
        else
        {
            Debug.Log(request.downloadHandler.text);
            EventManager m = EventManager.Instance;
            m.Endpoint = $"{request.downloadHandler.text}";
            m.StartClient();
        }
    }
}
