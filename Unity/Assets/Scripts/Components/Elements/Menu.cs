using System;
using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using UnityEngine;

namespace ARSIS.UI
{
    public class Menu : MonoBehaviour, IRenderable
    {
        private const string CommandKey = "hololens_command";

        private static readonly Dictionary<string, string> WindowPrefabPaths = new(StringComparer.OrdinalIgnoreCase)
        {
            { "biometrics", "prefabs/Biometrics" },
            { "navigation", "prefabs/Navigation" },
            { "procedures", "prefabs/ProceduresWindow" },
            { "spectrometry", "prefabs/Spectrometry" },
            { "notifications", "prefabs/NotificationWindow" },
            { "settings", "prefabs/SettingsWindow" },
            { "summary_timeline", "prefabs/SummaryTimeline" },
        };

        private static readonly Dictionary<string, GameObject> WindowInstances = new(StringComparer.OrdinalIgnoreCase);
        private static Menu commandHandlerOwner;

        private bool changed;
        private int processedCommandCount;
        private List<BaseArsisEvent> commandEvents = new();

        public void InstantiatePrefab(GameObject prefab)
        {
            OpenPrefab(prefab, NormalizeWindowName(prefab != null ? prefab.name : null));
        }

        public void Render(List<BaseArsisEvent> data)
        {
            commandEvents = data;
            changed = true;
        }

        void Update()
        {
            if (!changed) return;
            changed = false;
            ProcessQueuedCommands();
        }

        void Start()
        {
            if (commandHandlerOwner != null && commandHandlerOwner != this)
                return;

            commandHandlerOwner = this;
            EventDatastore.Instance.AddHandler(CommandKey, this);
        }

        void OnDestroy()
        {
            if (commandHandlerOwner != this)
                return;

            EventDatastore.Instance.RemoveHandler(CommandKey, this);
            commandHandlerOwner = null;
            DestroyAllWindowInstances();
        }

        private void DestroyAllWindowInstances()
        {
            HashSet<GameObject> instances = new(WindowInstances.Values);
            WindowInstances.Clear();

            foreach (GameObject instance in instances)
            {
                if (instance != null)
                    Destroy(instance);
            }
        }

        private void ProcessQueuedCommands()
        {
            if (processedCommandCount > commandEvents.Count)
                processedCommandCount = 0;

            for (int i = processedCommandCount; i < commandEvents.Count; i++)
            {
                if (commandEvents[i] is HololensCommand command)
                    HandleHololensCommand(command);
            }

            processedCommandCount = commandEvents.Count;
        }

        private void HandleHololensCommand(HololensCommand command)
        {
            if (IsStaleCommand(command))
                return;

            HololensCommandData data = command.data;
            if (data == null || string.IsNullOrWhiteSpace(data.action))
                return;

            switch (data.action)
            {
                case "open_window":
                    OpenWindow(data.window);
                    break;
                case "close_window":
                    CloseWindow(data.window);
                    break;
                case "open_procedure":
                    OpenProcedure(data.procedure);
                    break;
                default:
                    Debug.LogWarning($"Menu: Unsupported hololens command action '{data.action}'.");
                    break;
            }
        }

        private bool IsStaleCommand(HololensCommand command)
        {
            ARSIS.EventManager.EventManager eventManager = ARSIS.EventManager.EventManager.Instance;
            if (eventManager == null || eventManager.ApplicationStartTimeNs == 0)
                return false;

            if (command.time > eventManager.ApplicationStartTimeNs)
                return false;

            Debug.Log($"Discarding stale hololens command event from before app start: {command.data?.action}");
            return true;
        }

        private GameObject OpenWindow(string windowName)
        {
            string id = NormalizeWindowName(windowName);
            if (!WindowPrefabPaths.TryGetValue(id, out string prefabPath))
            {
                Debug.LogWarning($"Menu: Unknown hololens window '{windowName}'.");
                return null;
            }

            GameObject prefab = Resources.Load<GameObject>(prefabPath);
            if (prefab == null)
            {
                Debug.LogWarning($"Menu: Could not load window prefab at Resources/{prefabPath}.");
                return null;
            }

            return OpenPrefab(prefab, id);
        }

        private void CloseWindow(string windowName)
        {
            string id = NormalizeWindowName(windowName);
            if (WindowInstances.TryGetValue(id, out GameObject instance))
            {
                if (instance != null)
                {
                    DestroyWindowInstance(instance);
                    return;
                }

                WindowInstances.Remove(id);
            }

            if (WindowPrefabPaths.TryGetValue(id, out string prefabPath))
            {
                GameObject prefab = Resources.Load<GameObject>(prefabPath);
                if (FloatingMenuFromPrefab.Close(prefab))
                    return;
            }

            Debug.LogWarning($"Menu: No open hololens window found for '{windowName}'.");
        }

        private GameObject OpenPrefab(GameObject prefab, string windowId)
        {
            if (prefab == null)
            {
                Debug.Log("FAILED TO LOAD PREFAB");
                return null;
            }

            GameObject instance = prefab.GetComponentInChildren<FloatingMenuFromPrefab>(true) != null
                ? FloatingMenuFromPrefab.OpenOrFocus(prefab)
                : Instantiate(prefab);

            RegisterWindowInstance(windowId, prefab, instance);
            return instance;
        }

        private void DestroyWindowInstance(GameObject instance)
        {
            List<string> keysToRemove = new();
            foreach (KeyValuePair<string, GameObject> entry in WindowInstances)
            {
                if (entry.Value == instance)
                    keysToRemove.Add(entry.Key);
            }

            foreach (string key in keysToRemove)
                WindowInstances.Remove(key);

            Destroy(instance);
        }

        private void RegisterWindowInstance(string windowId, GameObject prefab, GameObject instance)
        {
            if (instance == null)
                return;

            if (!string.IsNullOrEmpty(windowId))
                WindowInstances[windowId] = instance;

            string prefabId = NormalizeWindowName(prefab != null ? prefab.name : null);
            if (!string.IsNullOrEmpty(prefabId))
                WindowInstances[prefabId] = instance;
        }

        private void OpenProcedure(string procedureName)
        {
            OpenWindow("procedures");
            StartCoroutine(OpenProcedureAfterWindowCreated(procedureName));
        }

        private IEnumerator OpenProcedureAfterWindowCreated(string procedureName)
        {
            yield return null;

            Procedures procedures = FindObjectOfType<Procedures>();
            if (procedures == null)
            {
                Debug.LogWarning("Menu: Procedures window is not available to open a procedure.");
                yield break;
            }

            procedures.OpenProcedureByName(procedureName);
        }

        private static string NormalizeWindowName(string windowName)
        {
            if (string.IsNullOrWhiteSpace(windowName))
                return string.Empty;

            string id = windowName.Trim().ToLowerInvariant().Replace(" ", "_").Replace("-", "_");
            switch (id)
            {
                case "map":
                    return "navigation";
                case "procedure":
                case "procedureswindow":
                    return "procedures";
                case "spectrometrywindow":
                    return "spectrometry";
                case "notification":
                case "notificationswindow":
                case "notificationwindow":
                    return "notifications";
                case "settingswindow":
                    return "settings";
                case "summarytimeline":
                case "timeline":
                case "eva_summary_timeline":
                    return "summary_timeline";
                default:
                    return id;
            }
        }
    }
}
