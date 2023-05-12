using System.Collections;
using System.Collections.Generic;
using SystemTasks = System.Threading.Tasks;
using UnityEngine;
using UnityEditor;

#if UNITY_EDITOR

[CustomEditor(typeof(LunaSARManagerEditor))]

public class LunaSARManagerEditor : Editor
{
    public override async void OnInspectorGUI()
    {
        LunaSARManager myTarget = (LunaSARManager)target;

        if (GUILayout.Button("Get Request To Route"))
        {
            myTarget.SendLSAR();
        }

        DrawDefaultInspector();
    }
}

#endif