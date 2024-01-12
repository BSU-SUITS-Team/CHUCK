using UnityEngine;
using TMPro;

[RequireComponent(typeof(RectTransform))]
[RequireComponent(typeof(TMP_Text))]
public class BoxSizeController : MonoBehaviour
{
    public float maxWidth = 200f;  // Set your maximum width here.

    private RectTransform rectTransform;
    private TMP_Text text;

    private void Awake()
    {
        rectTransform = GetComponent<RectTransform>();
        text = GetComponent<TMP_Text>();
    }

    private void Update()
    {
        text.ForceMeshUpdate();
        var preferredSize = text.GetPreferredValues(maxWidth, float.PositiveInfinity);
        rectTransform.sizeDelta = new Vector2(Mathf.Min(maxWidth, preferredSize.x), preferredSize.y *1.05f);
    }
}
