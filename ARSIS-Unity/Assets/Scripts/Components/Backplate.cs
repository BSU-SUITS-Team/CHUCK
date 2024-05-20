using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.GraphicsTools;
using UnityEngine;
using UnityEngine.UI;

namespace ARSIS.UI
{
    public class Backplate : IComponent
    {
        public string Name { get; set; } = "Backplate";
        private ThemeProvider theme;
        private ThemeProvider.Components component = ThemeProvider.Components.backplate;
        private ThemeProvider.Materials material = ThemeProvider.Materials.plate;

        public Backplate()
        {
            this.theme = ThemeProvider.Instance;
        }

        public Backplate(string name)
        {
            this.Name = name;
            this.theme = ThemeProvider.Instance;
        }

        public GameObject Build()
        {
            GameObject backplate = Object.Instantiate(theme.GetComponent(component));
            CanvasElementRoundedRect roundedRect = backplate.GetComponent<CanvasElementRoundedRect>();
            roundedRect.material = theme.GetMaterial(material);
            return backplate;
        }
    }
}
