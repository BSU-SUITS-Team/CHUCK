using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TextMessageSetter : MonoBehaviour, IMessageSetter
{
    [SerializeField] private TMPro.TMP_Text _text;

    public void SetMessage(string message)
    {
        _text.text = message;
    }
}
