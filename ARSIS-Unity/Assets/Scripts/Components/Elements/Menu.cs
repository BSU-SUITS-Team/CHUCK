using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace ARSIS.UI
{
    public class Menu : MonoBehaviour
    {
        [SerializeField]
        public Camera camera;
        public bool performBuild;
        public bool performDestroy;

        GameObject vertical;
        GameObject horizontal;
        IComponent notifications;
        IComponent pinned;
        IComponent menu;
        static float width = 200f;
        static float height = 300f;
        static float uiScale = 0.001f;
        static int padding = 10;
        static int spacing = 5;
        TextAnchor verticalAlign = TextAnchor.UpperLeft;
        static float notifPlateHeight = 60f;
        static float horizontalX = width - (padding * 2f);
        static float horizontalY = height - (padding * 2f) - notifPlateHeight - spacing;

        private void CreateComponents()
        {
            Backplate wideBackplate = new Backplate("Notifications");
            wideBackplate.Size = new Vector2(horizontalX, notifPlateHeight);
            notifications = wideBackplate;

            float plateWidth = (horizontalX - spacing) / 2f;
            GridLayoutGroup.Constraint constraint = GridLayoutGroup.Constraint.FixedColumnCount;
            int numColumns = 2;
            int numRows = 3;
            float buttonWidth = (plateWidth - (spacing * (numColumns + 1))) / numColumns;
            float buttonHeight = (horizontalY - (spacing * (numRows + 1))) / numRows;

            Backplate tallBackplate = new Backplate("tall");
            tallBackplate.Size = new Vector2(plateWidth, horizontalY);
            tallBackplate.Constraint = constraint;
            tallBackplate.Padding = new RectOffset(spacing, spacing, spacing, spacing);
            tallBackplate.Spacing = new Vector2(spacing, spacing);
            tallBackplate.CellSize = new Vector2(buttonWidth, buttonHeight);
            pinned = tallBackplate;
            menu = tallBackplate;
        }

        private void BuildLayouts()
        {
            vertical = new GameObject("VerticalLayout");
            vertical.transform.SetParent(gameObject.transform, false);
            RectTransform vtransform = vertical.AddComponent<RectTransform>();
            VerticalLayoutGroup vlayout = vertical.AddComponent<VerticalLayoutGroup>();
            Canvas vcanvas = vertical.AddComponent<Canvas>();
            vcanvas.renderMode = RenderMode.WorldSpace;
            vcanvas.worldCamera = camera;
            vtransform.sizeDelta = new Vector2(width, height); ;
            vtransform.localScale = Vector3.one * uiScale;
            vlayout.childControlWidth = false;
            vlayout.childControlHeight = false;
            vlayout.childForceExpandWidth = false;
            vlayout.childForceExpandHeight = false;
            vlayout.padding = new RectOffset(padding, padding, padding, padding);
            vlayout.spacing = spacing;
            vlayout.childAlignment = verticalAlign;

            horizontal = new GameObject("HorizontalLayout");
            horizontal.transform.SetParent(vertical.transform, false);
            RectTransform htransform = horizontal.AddComponent<RectTransform>();
            HorizontalLayoutGroup hlayout = horizontal.AddComponent<HorizontalLayoutGroup>();
            htransform.sizeDelta = new Vector2(horizontalX, horizontalY);
            hlayout.spacing = spacing;
            hlayout.childControlWidth = false;
            hlayout.childControlHeight = false;
            hlayout.childForceExpandWidth = false;
            hlayout.childForceExpandHeight = false;
        }

        private void BuildComponents()
        {
            GameObject notificationComponent = notifications.Build();
            notificationComponent.transform.SetParent(vertical.transform, false);
            GameObject pinnedComponent = pinned.Build();
            pinnedComponent.transform.SetParent(horizontal.transform, false);
            pinnedComponent.name = "Pinned";
            GameObject menuComponent = menu.Build();
            menuComponent.transform.SetParent(horizontal.transform, false);
            menuComponent.name = "Menu";
        }

        private void Build()
        {
            CreateComponents();
            BuildLayouts();
            BuildComponents();
        }

        // Start is called before the first frame update
        void Start()
        {
            Build();
        }

        // Update is called once per frame
        void Update()
        {
            if (Application.isPlaying) return;
            if (performBuild)
            {
                Build();
            }
            if (performDestroy)
            {
                DestroyImmediate(vertical);
            }

            performBuild = false;
            performDestroy = false;
        }
    }
}