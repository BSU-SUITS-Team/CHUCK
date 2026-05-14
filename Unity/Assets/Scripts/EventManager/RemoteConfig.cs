using System.Collections;
using UnityEngine.Networking;
using UnityEngine;
using ARSIS.EventManager;

// This is mostly for setting the remote URLs used to talk to the TSS and MCP servers.
public class RemoteConfig : MonoBehaviour
{
    private static string URL = "https://config.suits.dev.plsignore.me/servers";

    [System.Serializable]
    private class ServerConfig
    {
        public string TSS;
        public string MCP;
    }

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(GetServerConfig());
    }

    IEnumerator GetServerConfig()
    {
        UnityWebRequest request = UnityWebRequest.Get(URL);
        yield return request.SendWebRequest();

        if (request.result != UnityWebRequest.Result.Success)
        {
            Debug.Log(request.error);
        }
        else
        {
            string responseBody = request.downloadHandler.text;
            Debug.Log(responseBody);

            ServerConfig config = JsonUtility.FromJson<ServerConfig>(responseBody);
            if (config == null)
            {
                Debug.LogError("Failed to parse remote server config.");
                yield break;
            }

            if (!string.IsNullOrWhiteSpace(config.MCP))
            {
                EventManager manager = EventManager.Instance;
                if (manager != null)
                {
                    manager.Endpoint = BuildMcpEndpoint(config.MCP.Trim());
                    manager.StartClient();
                }
                else
                {
                    Debug.LogError("RemoteConfig: no EventManager found.");
                }
            }

            if (!string.IsNullOrWhiteSpace(config.TSS))
            {
                TSSConnectionManager tssManager = TSSConnectionManager.Instance;
                if (tssManager != null)
                {
                    tssManager.Connect(config.TSS.Trim());
                }
                else
                {
                    Debug.LogError("RemoteConfig: no TSSConnectionManager found.");
                }
            }
        }
    }

    private string BuildMcpEndpoint(string mcpHost)
    {
        string lowerMcpHost = mcpHost.ToLowerInvariant();
        if (lowerMcpHost.StartsWith("ws://") || lowerMcpHost.StartsWith("wss://"))
        {
            return mcpHost;
        }

        return $"ws://{mcpHost}:8181/ws/events";
    }
}
