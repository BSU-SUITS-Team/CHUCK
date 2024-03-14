using MixedReality.Toolkit;
using MixedReality.Toolkit.UX;
using UnityEngine;
using UnityEngine.UI;

namespace ARSIS.UI
{
    [ExecuteInEditMode]
    public class Menu : MonoBehaviour
    {
        [SerializeField]
        public Camera camera;
        public bool performBuild;
        public bool performDestroy;

        [SerializeField]
        FieldNotes fieldNotes;

        private GameObject vertical;
        private GameObject horizontal;
        private GameObject notifications;
        private GameObject pinned;
        private GameObject menu;
        private static float width = 300f;
        private static float height = 300f;
        private static float uiScale = 0.001f;
        private static int padding = 10;
        private static int spacing = 5;
        private TextAnchor verticalAlign = TextAnchor.UpperLeft;
        private static float notifPlateHeight = height / 4;
        private static float horizontalX = width - (padding * 2f);
        private static float horizontalY = height - (padding * 2f) - notifPlateHeight - spacing;
        private static string[] buttons = { 
            "Procedures", 
            "Biometrics", 
            "Spectrometry", 
            "Field Notes", 
            "Navigation", 
            "Settings", 
        };


        private void FormatBackplateObjects()
        {
            notifications.GetComponent<LayoutElement>().ignoreLayout = false;
            pinned.GetComponent<LayoutElement>().ignoreLayout = false;
            menu.GetComponent<LayoutElement>().ignoreLayout = false;

            notifications.GetComponent<RectTransform>().sizeDelta = new Vector2(horizontalX, notifPlateHeight);

            float plateWidth = (horizontalX - spacing) / 2f;
            GridLayoutGroup.Constraint constraint = GridLayoutGroup.Constraint.FixedColumnCount;
            int numColumns = 2;
            int numRows = 3;
            float buttonWidth = (plateWidth - (spacing * (numColumns + 1))) / numColumns;
            float buttonHeight = (horizontalY - (spacing * (numRows + 1))) / numRows;

            RectTransform menuTransform = menu.GetComponent<RectTransform>();
            menuTransform.sizeDelta = new Vector2(plateWidth, horizontalY);
            GridLayoutGroup menuGrid = menu.AddComponent<GridLayoutGroup>();
            menuGrid.constraint = constraint;
            menuGrid.constraintCount = numColumns;
            menuGrid.padding = new RectOffset(spacing, spacing, spacing, spacing);
            menuGrid.spacing = new Vector2(spacing, spacing);
            menuGrid.cellSize = new Vector2(buttonWidth, buttonHeight);
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
            hlayout.childAlignment = TextAnchor.UpperCenter;
            hlayout.childForceExpandWidth = true;
            hlayout.childControlWidth = true;
            hlayout.spacing = spacing;
            hlayout.childControlWidth = false;
            hlayout.childControlHeight = false;
            hlayout.childForceExpandWidth = false;
            hlayout.childForceExpandHeight = false;
        }

        private void BuildBackplateComponents()
        {
            Backplate backplateBuilder = new Backplate();
            notifications = backplateBuilder.Build();
            notifications.transform.SetParent(vertical.transform, false);
            notifications.transform.SetAsFirstSibling();
            pinned = backplateBuilder.Build();
            pinned.transform.SetParent(horizontal.transform, false);
            pinned.name = "Pinned";
            menu = backplateBuilder.Build();
            menu.transform.SetParent(horizontal.transform, false);
            menu.name = "Menu";
            FormatBackplateObjects();
        }

        private void BuildMenuButtons()
        {
            if (menu == null) return;
            Button buttonBuilder = new Button("button");
            buttonBuilder.HasIcon = false;
            buttonBuilder.HasText = true;
            foreach (string name in buttons)
            {
                buttonBuilder.Text = name;
                buttonBuilder.Build().transform.SetParent(menu.transform, false);
            }
            GameObject fieldNotes = menu.transform.GetChild(3).gameObject;
            StatefulInteractable pressable = fieldNotes.GetComponent<PressableButton>();
            pressable.OnClicked.AddListener(ShowFieldNotes);
        }

        private void ShowFieldNotes()
        {
            if (fieldNotes == null) return;
            fieldNotes.ShowFieldNotes(true);
        }

        private void Build()
        {
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

        // Start is called before the first frame update
        void Start()
        {
            DestroyAllChilds();
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
    }
}