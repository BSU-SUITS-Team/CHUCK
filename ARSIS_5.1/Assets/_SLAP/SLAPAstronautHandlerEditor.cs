using UnityEngine;
using UnityEditor;
using System.Collections;

#if UNITY_EDITOR

[CustomEditor(typeof(SLAPAstronautHandler))]
public class SLAPAstronautHandlerEditor : Editor
{
    override public void OnInspectorGUI()
    {
        

        DrawDefaultInspector();
    }
}

#endif 
