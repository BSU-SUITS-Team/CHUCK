using MixedReality.Toolkit;
using MixedReality.Toolkit.UX;
using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using UnityEngine;
using UnityEngine.UI;
using UnityEditor;

namespace ARSIS.UI
{
    [ExecuteInEditMode]
    public class Menu : MonoBehaviour, IRenderable
    {
        private string key = "telemetry";
        private bool changed = true;
        private List<BaseArsisEvent> data;

        public void InstantiatePrefab(GameObject prefab)
        {
            if (prefab == null)
            {
                Debug.Log("FAILED TO LOAD PREFAB");
                return;
            }
            Instantiate(prefab);
        }

        public void Render(List<BaseArsisEvent> data)
        {
            this.data = data;
            changed = true;
        }

        void Update()
        {
            if (!changed) return;
            changed = false;
        }

        void Start()
        {
            EventDatastore eventDatastore = EventDatastore.Instance;
            eventDatastore.AddHandler(key, this);
        }

        void OnDestroy()
        {
            EventDatastore eventDatastore = EventDatastore.Instance;
            eventDatastore.RemoveHandler(key, this);
        }

    }
}