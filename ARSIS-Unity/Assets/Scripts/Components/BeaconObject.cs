using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class BeaconObject : MonoBehaviour
{
    [SerializeField] Text label;
    [SerializeField] Text distance;

    public void SetText(string text)
    {
        label.text = text;
    }

    public void SetDistance(string text)
    {
        distance.text = text;
    }
}
