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
            plate = 0,
        }
        private Dictionary<Materials, Material> materials = new() { // Dictionary of name and path
            { Materials.plate, Resources.Load<Material>("materials/CanvasBackplate") }
        };

        private ThemeProvider() { }


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

        public void SetMaterial(Materials key, Material path)
        {
            materials[key] = path;
        }

        private class Nested
        {
            static Nested() { }

            internal static readonly ThemeProvider instance = new();
        }
    }
}
