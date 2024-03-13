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
        public Vector2 Size { get; set; } = new(100f, 100f);
        public Vector2 CellSize { get; set; } = new(50f, 50f);
        public GridLayoutGroup.Constraint Constraint { get; set; } = GridLayoutGroup.Constraint.Flexible;
        public int ConstraintCount { get; set; } = 0;
        public RectOffset Padding { get; set; } = new(5, 5, 5, 5);
        public Vector2 Spacing { get; set; } = new(5f, 5f);

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
