using System;
using System.Collections;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;
using UnityEngine.UI;
using TMPro;

public class TSSConnectionManager : MonoBehaviour
{
    public static TSSConnectionManager Instance { get; private set; }

    [Header("Connection")]
    [SerializeField] private string host = "";
    [SerializeField] private string port = "14141";
    [SerializeField] private float pollIntervalSeconds = 0.1f;
    [SerializeField] private bool connectOnStart = false;
    [SerializeField] private TMP_Text connectionDebugText;

    public bool IsConnected { get; private set; }
    public string CurrentHost => host;
    public string BaseUrl => string.IsNullOrWhiteSpace(host) ? "" : $"http://{host}:{port}";

    [Header("Debug / Cached Raw JSON")]
    [TextArea(3, 10)]
    [SerializeField] private string evaJson;

    public string EvaJson => evaJson;
    public EvaRoot EvaData { get; private set; }

    // Latched errors
    public ErrorLatch FanErrorLatch { get; } = new ErrorLatch();
    public ErrorLatch OxyErrorLatch { get; } = new ErrorLatch();
    public ErrorLatch PowerErrorLatch { get; } = new ErrorLatch();
    public ErrorLatch ScrubberErrorLatch { get; } = new ErrorLatch();

    // Events
    public event Action<bool> ConnectionChanged;
    public event Action<EvaRoot> EvaUpdated;
    public event Action ErrorLatchUpdated;

    private Coroutine pollRoutine;
    private bool pollInFlight;

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;
        DontDestroyOnLoad(gameObject);
    }

    private void Start()
    {
        if (connectOnStart && !string.IsNullOrWhiteSpace(host))
        {
            Connect(host);
        }
    }

    private void OnDestroy()
    {
        if (Instance == this)
        {
            Disconnect();
            Instance = null;
        }
    }

    public void Connect(string newHost)
    {
        if (string.IsNullOrWhiteSpace(newHost))
        {
            Debug.LogWarning("TSS connect skipped: host is empty.");
            return;
        }

        host = newHost.Trim();

        if (pollRoutine != null)
        {
            StopCoroutine(pollRoutine);
            pollRoutine = null;
        }

        IsConnected = false;
        pollInFlight = false;
        ConnectionChanged?.Invoke(false);

        StartCoroutine(TestConnectionAndStartPolling());
    }

    public void Disconnect()
    {
        if (pollRoutine != null)
        {
            StopCoroutine(pollRoutine);
            pollRoutine = null;
        }

        IsConnected = false;
        pollInFlight = false;
        ConnectionChanged?.Invoke(false);
    }

    private IEnumerator TestConnectionAndStartPolling()
    {
        using UnityWebRequest request = UnityWebRequest.Get(BaseUrl);
        yield return request.SendWebRequest();

        if (request.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError($"TSS connection test failed: {request.error} ({BaseUrl})");
            connectionDebugText.text = $"TSS connection test failed: {request.error} ({BaseUrl})";
            IsConnected = false;
            ConnectionChanged?.Invoke(false);
            yield break;
        }

        Debug.Log($"TSS connected: {BaseUrl}");
        connectionDebugText.text = $"TSS connected: {BaseUrl}";
        IsConnected = true;
        ConnectionChanged?.Invoke(true);

        pollRoutine = StartCoroutine(PollLoop());
    }

    private IEnumerator PollLoop()
    {
        WaitForSeconds wait = new WaitForSeconds(pollIntervalSeconds);

        while (IsConnected)
        {
            if (!pollInFlight)
            {
                yield return StartCoroutine(PollEva());
            }

            yield return wait;
        }
    }

    private IEnumerator PollEva()
    {
        pollInFlight = true;

        using UnityWebRequest request = UnityWebRequest.Get(BaseUrl + "/data/EVA.json");
        yield return request.SendWebRequest();

        if (request.result != UnityWebRequest.Result.Success)
        {
            Debug.LogWarning($"TSS EVA fetch failed: {request.error}");
            pollInFlight = false;
            yield break;
        }

        string newJson = request.downloadHandler.text;

        if (newJson != evaJson)
        {
            evaJson = newJson;

            try
            {
                EvaRoot parsed = JsonConvert.DeserializeObject<EvaRoot>(newJson);
                EvaData = parsed;

                UpdateErrorLatches(parsed);

                EvaUpdated?.Invoke(EvaData);
                ErrorLatchUpdated?.Invoke();
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to parse EVA JSON: {ex.Message}");
            }
        }

        pollInFlight = false;
    }

    private void UpdateErrorLatches(EvaRoot data)
    {
        bool fanLive = data?.error != null && data.error.fan_error;
        bool oxyLive = data?.error != null && data.error.oxy_error;
        bool powerLive = data?.error != null && data.error.power_error;
        bool scrubberLive = data?.error != null && data.error.scrubber_error;

        FanErrorLatch.UpdateFromLive(fanLive);
        OxyErrorLatch.UpdateFromLive(oxyLive);
        PowerErrorLatch.UpdateFromLive(powerLive);
        ScrubberErrorLatch.UpdateFromLive(scrubberLive);
    }

    // Reset methods
    public void ResetFanError()
    {
        FanErrorLatch.Reset();
        ErrorLatchUpdated?.Invoke();
    }

    public void ResetOxyError()
    {
        OxyErrorLatch.Reset();
        ErrorLatchUpdated?.Invoke();
    }

    public void ResetPowerError()
    {
        PowerErrorLatch.Reset();
        ErrorLatchUpdated?.Invoke();
    }

    public void ResetScrubberError()
    {
        ScrubberErrorLatch.Reset();
        ErrorLatchUpdated?.Invoke();
    }

    public void ResetAllErrors()
    {
        FanErrorLatch.Reset();
        OxyErrorLatch.Reset();
        PowerErrorLatch.Reset();
        ScrubberErrorLatch.Reset();
        ErrorLatchUpdated?.Invoke();
    }

    // Optional helper getters
    public float GetEva1BatteryPercent()
    {
        if (EvaData?.telemetry?.eva1 == null) return 0f;
        return EvaData.telemetry.eva1.primary_battery_level;
    }

    public float GetEva2BatteryPercent()
    {
        if (EvaData?.telemetry?.eva2 == null) return 0f;

        if (EvaData.telemetry.eva2.battery_level != 0f)
            return EvaData.telemetry.eva2.battery_level;

        return EvaData.telemetry.eva2.primary_battery_level;
    }

    public float GetEva1Temperature()
    {
        if (EvaData?.telemetry?.eva1 == null) return 0f;
        return EvaData.telemetry.eva1.temperature;
    }

    public float GetEva2Temperature()
    {
        if (EvaData?.telemetry?.eva2 == null) return 0f;
        return EvaData.telemetry.eva2.temperature;
    }

    public bool GetEvaStarted()
    {
        return EvaData?.status != null && EvaData.status.started;
    }
}

[Serializable]
public class ErrorLatch
{
    public bool live;
    public bool latched;
    public float trippedAtTime = -1f;

    public float SecondsSinceTrip =>
        latched && trippedAtTime >= 0f
            ? Time.unscaledTime - trippedAtTime
            : 0f;

    public void UpdateFromLive(bool isLive)
    {
        live = isLive;

        if (isLive && !latched)
        {
            latched = true;
            trippedAtTime = Time.unscaledTime;
        }
    }

    public void Reset()
    {
        live = false;
        latched = false;
        trippedAtTime = -1f;
    }
}