#if UNITY_EDITOR

using UnityEngine;
using UnityEditor;
using System.Collections;

[CustomEditor(typeof(MapManager))]
public class MapManagerEditor : Editor
{
    override public void OnInspectorGUI()
    {
        DrawDefaultInspector();
    }
}

#endif
