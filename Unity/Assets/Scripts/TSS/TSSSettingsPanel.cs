using TMPro;
using UnityEngine;
using UnityEngine.UI;
using MixedReality.Toolkit.UX;

public class TSSSettingsPanel : MonoBehaviour
{
    [SerializeField] private MRTKUGUIInputField hostInput;
    [SerializeField] private TMP_Text connectionStatusText;
    [SerializeField] private TMP_Text connectionIPText;

    private TSSConnectionManager Manager => TSSConnectionManager.Instance;

    private void OnEnable()
    {
        if (Manager == null)
        {
            Debug.LogError("TSSSettingsPanel: no TSSConnectionManager found.");
            return;
        }

        Manager.ConnectionChanged += HandleConnectionChanged;

        if (hostInput != null && !string.IsNullOrEmpty(Manager.CurrentHost))
        {
            hostInput.text = Manager.CurrentHost;
        }

        HandleConnectionChanged(Manager.IsConnected);
    }

    private void OnDisable()
    {
        if (Manager != null)
        {
            Manager.ConnectionChanged -= HandleConnectionChanged;
        }
    }

    public void ConnectButton()
    {
        if (Manager == null)
        {
            Debug.LogError("TSSSettingsPanel: manager missing.");
            return;
        }

        string host = hostInput != null ? hostInput.text : "";
        connectionIPText.text = "TSS IP: " + host;
        Manager.Connect(host);
    }

    public void DisconnectButton()
    {
        if (Manager == null)
        {
            Debug.LogError("TSSSettingsPanel: manager missing.");
            return;
        }

        Manager.Disconnect();
    }

    private void HandleConnectionChanged(bool connected)
    {
        if (connectionStatusText != null)
        {
            connectionStatusText.text = connected ? "TSS: Connected" : "TSS: Disconnected";
        }
    }
}