using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.UI
{
    public sealed class ThemeProvider
    {
        public static ThemeProvider Instance { get { return Nested.instance; } }
        public enum Materials
        {
            plate,
        }
        private Dictionary<Materials, Material> materials = new() { // Dictionary of name and path
            { Materials.plate, Resources.Load<Material>("materials/CanvasBackplate") },
        };
        public enum Components
        {
            button,
            backplate,
            container,
        }
        private Dictionary<Components, GameObject> components = new() { // Dictionary of name and path
            { Components.button, Resources.Load<GameObject>("prefabs/Action Button") },
            { Components.backplate, Resources.Load<GameObject>("prefabs/Plate") },
            { Components.container, Resources.Load<GameObject>("prefabs/Container") },
        };

        private ThemeProvider() {}


        public Material GetMaterial(Materials key)
        {
            try
            {
                return materials[key];
            }
            catch (KeyNotFoundException e)
            {
                Debug.LogError(e);
                return null;
            }
        }

        public void SetMaterial(Materials key, Material material)
        {
            materials[key] = material;
        }

        public GameObject GetComponent(Components key)
        {
            try
            {
                return components[key];
            }
            catch (KeyNotFoundException e)
            {
                Debug.LogError(e);
                return null;
            }
        }

        public void SetComponent(Components key, GameObject component)
        {
            components[key] = component;
        }

        private class Nested
        {
            static Nested() { }

            internal static readonly ThemeProvider instance = new();
        }
    }
}
