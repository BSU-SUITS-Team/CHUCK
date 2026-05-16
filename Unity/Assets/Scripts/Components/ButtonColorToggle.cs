using UnityEngine;
using UnityEngine.UI;

public class ButtonColorToggle : MonoBehaviour
{

    public RawImage image;
    public Color firstColor;
    public Color secondColor;

    bool state = false;

    void Start()
    {
        ApplyColor();
    }

    public void ToggleColor() {
        state = !state;
        ApplyColor();
    }

    public void SetState(bool enabled)
    {
        state = enabled;
        ApplyColor();
    }

    void ApplyColor()
    {
        if (image == null)
        {
            Debug.LogWarning($"{nameof(ButtonColorToggle)} on {name} has no image assigned.", this);
            return;
        }

        image.color = state ? firstColor : secondColor;
    }
}
