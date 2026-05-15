using System.Collections;
using System.Text;
using UnityEngine;
using UnityEngine.Networking;

namespace ARSIS.EventManager
{
    public class VoiceTranscriptionToggleSender : MonoBehaviour
    {
        private const string TogglePath = "/voice/transcription/toggle";

        [Header("Command")]
        [SerializeField] private string source = "hololens";
        [SerializeField] private string target = "aia";

        [System.Serializable]
        private class VoiceTranscriptionToggleRequest
        {
            public string action = "toggle";
            public string target = "aia";
            public string source = "hololens";
        }

        [ContextMenu("Toggle Voice Transcription")]
        public void ToggleVoiceTranscription()
        {
            StartCoroutine(SendToggleRequest());
        }

        public void SetSource(string newSource)
        {
            source = string.IsNullOrWhiteSpace(newSource) ? "hololens" : newSource.Trim();
        }

        public void SetTarget(string newTarget)
        {
            target = string.IsNullOrWhiteSpace(newTarget) ? "aia" : newTarget.Trim();
        }

        private IEnumerator SendToggleRequest()
        {
            string endpoint = GetToggleEndpoint();
            string requestJson = JsonUtility.ToJson(new VoiceTranscriptionToggleRequest
            {
                target = string.IsNullOrWhiteSpace(target) ? "aia" : target.Trim(),
                source = string.IsNullOrWhiteSpace(source) ? "hololens" : source.Trim(),
            });

            byte[] body = Encoding.UTF8.GetBytes(requestJson);

            using UnityWebRequest request = new UnityWebRequest(endpoint, UnityWebRequest.kHttpVerbPOST);
            request.uploadHandler = new UploadHandlerRaw(body);
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");

            yield return request.SendWebRequest();

            if (request.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError($"Voice transcription toggle failed: {request.error} ({endpoint})");
                yield break;
            }

            Debug.Log($"Voice transcription toggle sent: {request.downloadHandler.text}");
        }

        private string GetToggleEndpoint()
        {
            return NormalizeGroundControlApiUrl(GetConfiguredApiUrl()) + TogglePath;
        }

        private string GetConfiguredApiUrl()
        {
            EventManager manager = EventManager.Instance;
            if (manager != null && !string.IsNullOrWhiteSpace(manager.Endpoint))
                return manager.Endpoint;

            return "http://localhost:8181";
        }

        private static string NormalizeGroundControlApiUrl(string urlOrHost)
        {
            if (string.IsNullOrWhiteSpace(urlOrHost))
                return "http://localhost:8181";

            string normalized = urlOrHost.Trim().TrimEnd('/');

            if (normalized.EndsWith(TogglePath))
                normalized = normalized.Substring(0, normalized.Length - TogglePath.Length);

            if (normalized.EndsWith("/ws/events"))
                normalized = normalized.Substring(0, normalized.Length - "/ws/events".Length);

            string lowerNormalized = normalized.ToLowerInvariant();

            if (lowerNormalized.StartsWith("ws://"))
                return "http://" + normalized.Substring("ws://".Length);

            if (lowerNormalized.StartsWith("wss://"))
                return "https://" + normalized.Substring("wss://".Length);

            if (lowerNormalized.StartsWith("http://") || lowerNormalized.StartsWith("https://"))
                return normalized;

            return normalized.Contains(":") ? $"http://{normalized}" : $"http://{normalized}:8181";
        }
    }
}
