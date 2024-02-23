using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.UI
{
    public sealed class ThemeProvider
    {
        public static ThemeProvider Instance { get { return Nested.instance; } }
        private Dictionary<string, Material> materials = new() {
            { "plate", Resources.Load("materials/CanvasBackplate.mat", typeof(Material)) as Material }
        };

        private ThemeProvider() { }


        public Material GetMaterial(string key)
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

        public void SetMaterial(string key, Material material)
        {
            materials.Add(key, material);
        }

        private class Nested
        {
            static Nested() { }

            internal static readonly ThemeProvider instance = new();
        }
    }
}
