using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.GraphicsTools;
using UnityEngine;
using UnityEngine.UI;

namespace ARSIS.UI
{
    public class Backplate : IComponent
    {
        private string name { get; set; }
        private ThemeProvider theme;
        private Vector2 cellSize { get; set; }
        private GridLayoutGroup.Constraint constraint { get; set; }
        private int constraintCount { get; set; }
        private RectOffset padding { get; set; }
        private Vector2 spacing { get; set; }

        public Backplate(string name)
        {
            this.name = name;
            this.theme = ThemeProvider.Instance;
        }

        private void AddCanvas(GameObject gameObject)
        {
            CanvasRenderer renderer = gameObject.AddComponent<CanvasRenderer>();
            renderer.cullTransparentMesh = true;
        }

        private void AddPlate(GameObject gameObject)
        {
            CanvasElementRoundedRect plate = gameObject.AddComponent<CanvasElementRoundedRect>();
            plate.material = theme.GetMaterial("plate");
        }

        private void AddLayout(GameObject gameObject)
        {
            GridLayoutGroup grid = gameObject.AddComponent<GridLayoutGroup>();
            grid.cellSize = cellSize;
            grid.constraint = constraint;
            grid.constraintCount = constraintCount;
            grid.padding = padding;
            grid.spacing = spacing;
        }

        public GameObject build()
        {
            GameObject backplate = new GameObject(name);
            AddCanvas(backplate);
            AddPlate(backplate);
            AddLayout(backplate);
            return backplate;
        }
    }
}
