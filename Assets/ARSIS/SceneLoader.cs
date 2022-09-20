using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class SceneLoader : MonoBehaviour
{
    [Header("MRTK prefabs")]
    public GameObject[] mrtk_prefabs;
    public GameObject[] editor_only_prefabs;

    private void Start()
    {
        LoadMRTKPrefabs();
    }

    #region load functions
    void LoadMRTKPrefabs ()
    {
        foreach (GameObject g in mrtk_prefabs)
        {
            Instantiate(g);
        }
        #if UNITY_EDITOR
        foreach (GameObject g in editor_only_prefabs)
        {
            Instantiate(g);
        }
        #endif
    }
    #endregion
}
