using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.GraphicsTools;
using UnityEngine;
using UnityEngine.UI;

namespace ARSIS.UI
{
    public class Backplate : IComponent
    {
        public string Name { get; set; } = "Plate";
        private ThemeProvider theme;
        public Vector2 Size { get; set; } = new(100f, 100f);
        public Vector2 CellSize { get; set; } = new(50f, 50f);
        public GridLayoutGroup.Constraint Constraint { get; set; } = GridLayoutGroup.Constraint.Flexible;
        public int ConstraintCount { get; set; } = 0;
        public RectOffset Padding { get; set; } = new(5, 5, 5, 5);
        public Vector2 Spacing { get; set; } = new(5f, 5f);

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
            plate.material = theme.GetMaterial(ThemeProvider.Materials.plate);
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
