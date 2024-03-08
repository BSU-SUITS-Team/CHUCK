using System.Collections;
using System.Collections.Generic;
using MixedReality.Toolkit.UX;
using TMPro;
using UnityEngine;

namespace ARSIS.UI
{
    public class Button : IComponent
    {

        public string Name { get; set; }
        private ThemeProvider theme;
        private float offsetZ = 5f;
        private float depth = 25f;
        public string Text { get; set; } = "Button";
        public int FontSize { get; set; } = 8;

        public Button(string name)
        {
            this.Name = name;
            this.theme = ThemeProvider.Instance;
        }

        private void AddFrontplate(GameObject gameObject)
        {
            RectTransform transform = gameObject.GetComponent<RectTransform>();
            GameObject frontplate = new GameObject("Frontplate");
            frontplate.transform.SetParent(gameObject.transform, false);
            RectTransform rectTransform = frontplate.AddComponent<RectTransform>();
            frontplate.AddComponent<CanvasRenderer>();
            TextMeshProUGUI content = frontplate.AddComponent<TextMeshProUGUI>();
            rectTransform.transform.position = Vector3.back * offsetZ;
            rectTransform.sizeDelta = transform.sizeDelta;
            content.text = Text;
            content.fontSize = FontSize;
            content.alignment = TextAlignmentOptions.Center;
        }

        public GameObject Build()
        {
            GameObject button = new GameObject(Name);
            button.AddComponent<RectTransform>();
            button.AddComponent<PressableButton>();
            button.AddComponent<UGUIInputAdapter>();
            BoxCollider collider = button.AddComponent<BoxCollider>();
            RectTransformColliderFitter fitter = button.AddComponent<RectTransformColliderFitter>();
            fitter.ThisCollider = collider;
            collider.center = new Vector3(collider.center.x, collider.center.y, offsetZ);
            collider.size = new Vector3(collider.size.x, collider.size.y, depth);
            Animator animator = button.AddComponent<Animator>();
            animator.enabled = false;
            button.AddComponent<AudioSource>();
            button.AddComponent<StateVisualizer>();
            button.AddComponent<InteractablePulse>();
            AddFrontplate(button);
            return button;
        }
    }
}