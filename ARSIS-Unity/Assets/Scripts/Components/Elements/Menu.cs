using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using UnityEngine;
using UnityEngine.UI;

namespace ARSIS.UI
{
    [ExecuteInEditMode]
    public class Menu : MonoBehaviour, IRenderable
    {
        [SerializeField]
        public Camera camera;
        public bool performBuild;
        public bool performDestroy;

        EventDatastore eventDatastore = EventDatastore.Instance;
        GameObject vertical;
        GameObject horizontal;
        GameObject notifications;
        GameObject pinned;
        GameObject menu;
        IComponent wideBackplate;
        IComponent tallBackplate;
        static float width = 200f;
        static float height = 300f;
        static float uiScale = 0.001f;
        static int padding = 10;
        static int spacing = 5;
        TextAnchor verticalAlign = TextAnchor.UpperLeft;
        static float notifPlateHeight = 60f;
        static float horizontalX = width - (padding * 2f);
        static float horizontalY = height - (padding * 2f) - notifPlateHeight - spacing;
        static string[] buttons = { 
            "Procedures", 
            "Biometrics", 
            "Spectrometry", 
            "Drop Breadcrumb", 
            "Navigation", 
            "Settings", 
        };


        private void CreateBackplateComponents()
        {
            Backplate wideBackplate = new Backplate("Notifications");
            wideBackplate.Size = new Vector2(horizontalX, notifPlateHeight);
            this.wideBackplate = wideBackplate;

            float plateWidth = (horizontalX - spacing) / 2f;
            GridLayoutGroup.Constraint constraint = GridLayoutGroup.Constraint.FixedColumnCount;
            int numColumns = 2;
            int numRows = 3;
            float buttonWidth = (plateWidth - (spacing * (numColumns + 1))) / numColumns;
            float buttonHeight = (horizontalY - (spacing * (numRows + 1))) / numRows;

            Backplate tallBackplate = new Backplate("tall");
            tallBackplate.Size = new Vector2(plateWidth, horizontalY);
            tallBackplate.Constraint = constraint;
            tallBackplate.ConstraintCount = numColumns;
            tallBackplate.Padding = new RectOffset(spacing, spacing, spacing, spacing);
            tallBackplate.Spacing = new Vector2(spacing, spacing);
            tallBackplate.CellSize = new Vector2(buttonWidth, buttonHeight);
            this.tallBackplate = tallBackplate;
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

        private void BuildBackplateComponents()
        {
            notifications = wideBackplate.Build();
            notifications.transform.SetParent(vertical.transform, false);
            notifications.transform.SetAsFirstSibling();
            pinned = tallBackplate.Build();
            pinned.transform.SetParent(horizontal.transform, false);
            pinned.name = "Pinned";
            menu = tallBackplate.Build();
            menu.transform.SetParent(horizontal.transform, false);
            menu.name = "Menu";
        }

        private void BuildMenuButtons()
        {
            if (menu == null) return;
            Button button = new Button("button");
            foreach (string name in buttons)
            {
                button.Name = name;
                button.Text = name;
                GameObject buttonGameObject = button.Build();
                buttonGameObject.transform.SetParent(menu.transform, false);
            }
        }

        private void Build()
        {
            CreateBackplateComponents();
            BuildLayouts();
            BuildBackplateComponents();
            BuildMenuButtons();
        }

        private void DestroyAllChilds()
        {
            foreach (Transform transform in gameObject.transform)
            {
                DestroyImmediate(transform.gameObject);
            }
        }

        public void SetActive(bool active)
        {
            if (vertical == null) return;
            vertical.SetActive(active);
        }

        public void Render(List<BaseArsisEvent> events)
        {
            Debug.Log("MENU RENDER");
        }

        // Start is called before the first frame update
        void Start()
        {
            DestroyAllChilds();
            eventDatastore.AddHandler("telemetry", this);
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
                DestroyAllChilds();
            }

            performBuild = false;
            performDestroy = false;
        }

        public void Update(List<BaseArsisEvent> data)
        {
            throw new System.NotImplementedException();
        }
    }
}