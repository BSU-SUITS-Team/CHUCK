using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.GraphicsTools;
using UnityEngine;
using UnityEngine.UI;

namespace ARSIS.UI
{
    public class Backplate : IComponent
    {
        public string Name { get; set; }
        private ThemeProvider theme;
        public Vector2 Size { get; set; }
        public Vector2 CellSize { get; set; }
        public GridLayoutGroup.Constraint Constraint { get; set; }
        public int ConstraintCount { get; set; }
        public RectOffset Padding { get; set; }
        public Vector2 Spacing { get; set; }

        public Backplate(string name)
        {
            this.Name = name;
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
            grid.cellSize = CellSize;
            grid.constraint = Constraint;
            grid.constraintCount = ConstraintCount;
            grid.padding = Padding;
            grid.spacing = Spacing;
        }

        public GameObject Build()
        {
            GameObject backplate = new GameObject(Name);
            RectTransform transform = backplate.AddComponent<RectTransform>();
            transform.sizeDelta = Size;
            AddCanvas(backplate);
            AddPlate(backplate);
            AddLayout(backplate);
            return backplate;
        }
    }
}
