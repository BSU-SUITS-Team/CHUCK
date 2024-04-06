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
        public void InstantiatePrefab(string prefabName)
        {
            GameObject prefab = Resources.Load<GameObject>(prefabName);
            if (prefab == null)
            {
                Debug.Log("FAILED TO LOAD PREFAB: " + prefabName);
                return;
            }
            Instantiate(prefab);
        }

        public void Render(List<BaseArsisEvent> events)
        {
            Debug.Log("MENU RENDER");
        }

        void Awake()
        {
            EventDatastore.Instance.AddHandler("telemetry", this);
        }
    }
}