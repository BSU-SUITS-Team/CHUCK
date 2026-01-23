using System;
using System.Collections;
using System.Collections.Generic;
using MixedReality.Toolkit.UX;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

namespace ARSIS.UI
{
    public class Button : MonoBehaviour
    {
        [SerializeField] GameObject iconObject;
        [SerializeField] GameObject textObject;
        [SerializeField] FontIconSelector iconName;
        [SerializeField] TextMeshProUGUI iconLabel;
        [SerializeField] TextMeshProUGUI textValue;
        [SerializeField] PressableButton pressableButton;

        public PressableButton GetPressableButton()
        {
            return pressableButton;
        }

        public void SetIcon(string icon)
        {
            iconName.CurrentIconName = icon;
        }

        public void SetIcon(bool enabled, string icon, string label)
        {
            iconObject.SetActive(enabled);
            iconName.CurrentIconName = icon;
            iconLabel.text = label;
        }

        public void SetText(string text)
        {
            textValue.text = text;
        }

        public void SetText(bool enabled, string text)
        {
            textObject.SetActive(enabled);
            textValue.text = text;
        }
    }
}