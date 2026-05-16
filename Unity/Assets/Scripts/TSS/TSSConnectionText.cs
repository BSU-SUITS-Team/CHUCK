using ARSIS.EventManager;
using MixedReality.Toolkit.UX;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class TSSConnectionText : MonoBehaviour
{
    [SerializeField] MRTKUGUIInputField inputField;
    private TouchScreenKeyboard keyboard;

    public void OpenSystemKeyboardTSS()
    {
        keyboard = TouchScreenKeyboard.Open(inputField.text, TouchScreenKeyboardType.URL, false, false, false, false);
    }
}
