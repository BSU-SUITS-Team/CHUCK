using System.Collections;
using System.Collections.Generic;
using MixedReality.Toolkit.UX;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

namespace ARSIS.UI
{
    public class Button : IComponent
    {
        private static string DefaultIcon = "Icon 117";
        public string Name { get; set; } = "Button";
        private ThemeProvider theme;
        private ThemeProvider.Components key = ThemeProvider.Components.button;
        public bool HasIcon { get; set; } = true;
        public bool HasText { get; set; } = true;
        public string Text { get; set; } = string.Empty;
        public string IconName { get; set; } = DefaultIcon;

        public Button()
        {
            theme = ThemeProvider.Instance;
        }

        public Button(string Name)
        {
            this.Name = Name;
            theme = ThemeProvider.Instance;
        }

        void SetFrontplate(GameObject frontplate)
        {
            GameObject icon = frontplate.transform.GetChild(0).GetChild(0).gameObject;
            GameObject text = frontplate.transform.GetChild(0).GetChild(1).gameObject;
            icon.SetActive(HasIcon);
            text.SetActive(HasText);
            TextMeshProUGUI textComponent = text.GetComponent<TextMeshProUGUI>();
            if (HasText) textComponent.text = Text;
            if (HasIcon)
            {
                GameObject iconUI = icon.transform.GetChild(1).gameObject;
                FontIconSelector iconSelector = iconUI.GetComponent<FontIconSelector>();
                iconSelector.CurrentIconName = IconName ?? DefaultIcon;
            }
        }

        public GameObject Build()
        {
            GameObject button = Object.Instantiate(theme.GetComponent(key));
            button.name = Name;
            GameObject frontplate = button.transform.GetChild(2).gameObject;
            SetFrontplate(frontplate);
            return button;
        }
    }
}