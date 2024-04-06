using Microsoft.MixedReality.GraphicsTools;
using MixedReality.Toolkit.UX;
using MixedReality.Toolkit.UX.Experimental;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;

namespace ARSIS.UI
{
    [ExecuteInEditMode]
    public class FieldNotes : MonoBehaviour
    {
        private ThemeProvider.Components button = ThemeProvider.Components.button;

        public bool performBuild;
        public bool performDestroy;

        private static float uiScale = 0.001f;
        private static float width = 300f;
        private static float height = 300f;
        private static Vector2 size = new(width, height);
        private static float textOffsetZ = -4f;
        private static int padding = 5;
        private static int containerSpacing = 2;
        private static float headerHeight = 20f;
        private static float footerHeight = 30f;
        private static string headerTitle = "Field Notes";
        private static string pinIcon = "Icon 120";
        private static string closeIcon = "Icon 79";
        private static string createIcon = "Icon 112";
        private static string createText = "New Field Note";
        private static string deleteIcon = "Icon 76";
        private static string deleteText = "Delete All";
        private GameObject root;
        private GameObject backplate;
        private GameObject header;
        private GameObject list;
        private GameObject container;
        private GameObject footer;

        public void ShowFieldNotes(bool show)
        {
            if (root == null) return;
            root.GetComponent<Canvas>().enabled = show;
        }

        private void BuildLayout()
        {
            // create root
            root = new GameObject("Root");
            root.transform.SetParent(gameObject.transform, false);
            RectTransform rectTransform = root.AddComponent<RectTransform>();
            root.transform.localScale = Vector3.one * uiScale;
            rectTransform.sizeDelta = size;
            Canvas canvas = root.AddComponent<Canvas>();
            canvas.enabled = false;
            // create backplate
            Backplate backplateBuilder = new Backplate();
            backplate = backplateBuilder.Build();
            backplate.transform.SetParent(root.transform, false);
            RectTransform backplateRectTransform = backplate.GetComponent<RectTransform>();
            backplateRectTransform.sizeDelta = size;
        }

        private void BuildHeader()
        {
            // create header
            header = new GameObject("Header");
            header.AddComponent<RectTransform>();
            header.transform.SetParent(list.transform, false);
            HorizontalLayoutGroup headerLayout = header.AddComponent<HorizontalLayoutGroup>();
            LayoutElement headerElement = header.AddComponent<LayoutElement>();
            headerLayout.childControlWidth = true;
            headerLayout.childControlHeight = true;
            headerLayout.childForceExpandWidth = true;
            headerElement.preferredHeight = headerHeight;
            // create title
            GameObject title = new GameObject("Title");
            RectTransform titleRectTransform = title.AddComponent<RectTransform>();
            Vector3 adjustText = titleRectTransform.position;
            adjustText.z = textOffsetZ;
            titleRectTransform.position = adjustText;
            title.transform.SetParent(header.transform, false);
            TextMeshProUGUI titleText = title.AddComponent<TextMeshProUGUI>();
            titleText.enableAutoSizing = true;
            titleText.text = headerTitle;
            // create header buttons
            GameObject buttons = new GameObject("Buttons");
            buttons.AddComponent<RectTransform>();
            buttons.transform.SetParent(header.transform, false);
            HorizontalLayoutGroup buttonsLayout = buttons.AddComponent<HorizontalLayoutGroup>();
            buttonsLayout.spacing = padding;
            buttonsLayout.childControlHeight = true;
            buttonsLayout.childControlWidth = true;
            buttonsLayout.childForceExpandHeight = true; 
            buttonsLayout.childForceExpandWidth = true;
            buttonsLayout.childAlignment = TextAnchor.MiddleCenter;
            Button buttonBuilder = new Button();
            buttonBuilder.HasIcon = true;
            buttonBuilder.HasText = false;
            buttonBuilder.IconName = pinIcon;
            buttonBuilder.Build().transform.SetParent(buttons.transform, false);
            buttonBuilder.IconName = closeIcon;
            buttonBuilder.Build().transform.SetParent(buttons.transform, false);
        }

        private void BuildFooter()
        {
            // create footer
            footer = new GameObject("Footer");
            footer.AddComponent<RectTransform>();
            footer.transform.SetParent(list.transform, false);
            HorizontalLayoutGroup footerLayout = footer.AddComponent<HorizontalLayoutGroup>();
            LayoutElement footerElement = footer.AddComponent<LayoutElement>();
            footerLayout.childControlHeight = true;
            footerLayout.childControlWidth = true;
            footerLayout.childForceExpandHeight = true;
            footerLayout.childForceExpandWidth = true;
            footerLayout.childScaleWidth = true;
            footerLayout.childScaleHeight = true;
            footerLayout.childAlignment = TextAnchor.UpperCenter;
            footerElement.preferredHeight = footerHeight;
            // create footer buttons
            Button buttonBuilder = new Button();
            buttonBuilder.HasIcon = true;
            buttonBuilder.HasText = true;
            buttonBuilder.IconName = createIcon;
            buttonBuilder.Text = createText;
            buttonBuilder.Build().transform.SetParent(footer.transform, false);
            buttonBuilder.IconName = deleteIcon;
            buttonBuilder.Text = deleteText;
            buttonBuilder.Build().transform.SetParent(footer.transform, false);
        }

        private void BuildContainer()
        {
            RectTransform listRectTransform = list.GetComponent<RectTransform>();
          
            container = new GameObject("Container");
            container.AddComponent<RectTransform>();
            container.transform.SetParent(list.transform, false);
            LayoutElement containerElement = container.AddComponent<LayoutElement>();
            containerElement.minHeight = 0;
            containerElement.flexibleHeight = height - headerHeight - footerHeight;
            container.AddComponent<RectMask2DFast>();
            ScrollRect scrollRect = container.AddComponent<ScrollRect>();
            scrollRect.content = listRectTransform;
            scrollRect.viewport = listRectTransform;
            scrollRect.horizontal = true;
            scrollRect.vertical = false;
            scrollRect.movementType = ScrollRect.MovementType.Clamped;
            Scrollable scrollable = container.AddComponent<Scrollable>();
            scrollable.ScrollRect = scrollRect;
            VerticalLayoutGroup containerLayout = container.AddComponent<VerticalLayoutGroup>();
            containerLayout.childControlWidth = true;
            containerLayout.childControlHeight = true;
            containerLayout.childForceExpandWidth = true;
            containerLayout.childForceExpandHeight = false;
            containerLayout.spacing = containerSpacing;
            //VirtualizedScrollRectList scrollList = container.AddComponent<VirtualizedScrollRectList>();
            //scrollList.SetItemCount(5);
            //scrollList.OnInvisible = (prefab, index) =>
            //{
            //    GameObject frontplate = prefab.transform.GetChild(2).gameObject;
            //    GameObject icon = frontplate.transform.GetChild(0).GetChild(0).gameObject;
            //    GameObject text = frontplate.transform.GetChild(0).GetChild(1).gameObject;
            //    TextMeshProUGUI textComponent = text.GetComponent<TextMeshProUGUI>();
            //    textComponent.text = index.ToString();
            //    ShowFieldNotes(false);
            //};
            //scrollList.OnVisible = (prefab, index) =>
            //{
            //    GameObject frontplate = prefab.transform.GetChild(2).gameObject;
            //    GameObject icon = frontplate.transform.GetChild(0).GetChild(0).gameObject;
            //    GameObject text = frontplate.transform.GetChild(0).GetChild(1).gameObject;
            //    TextMeshProUGUI textComponent = text.GetComponent<TextMeshProUGUI>();
            //    textComponent.text = index.ToString();
            //    ShowFieldNotes(true);
            //};
            //scrollList.runInEditMode = true;
        }

        private void BuildList()
        {
            // create list
            list = new GameObject("List");
            RectTransform listRectTransform = list.AddComponent<RectTransform>();
            list.transform.SetParent(backplate.transform, false);
            listRectTransform.sizeDelta = size;
            VerticalLayoutGroup listLayout = list.AddComponent<VerticalLayoutGroup>();
            listLayout.padding = new RectOffset(padding, padding, padding, padding);
            listLayout.spacing = padding;
            listLayout.childControlWidth = true;
            listLayout.childControlHeight = true;
            listLayout.childForceExpandHeight = true;
            BuildHeader(); // create the header
            BuildContainer(); // create the container
            BuildFooter(); // create the footer
        }

        private void DestroyAllChilds()
        {
            foreach (Transform transform in gameObject.transform)
            {
                DestroyImmediate(transform.gameObject);
            }
        }

        private void Build() {
            BuildLayout();
            BuildList();
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
