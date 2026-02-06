using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class vertGaugeSplit1 : MonoBehaviour
{
    // split red-green zones at start-up
    public Transform needle; 
    public float minValue = 0f; 
    public float maxValue = 100f; 
    public float safeZoneValue = 60f;
    public float currentValue = 0f;

    private float needleYPosition = -20f;
    private float safeZoneYPosition = 0f;
    private float xPosition = 0f; 

    public Text labelText; 
    public Text valueText;
    public string labelTXT;
    public string units;

    public Color redColor = Color.red; 

    public RectTransform safeZoneLine; 
    public RectTransform greenArea; 
    private Image gaugeImage; 

    void Start()
    {
        gaugeImage = GetComponent<Image>();
        labelText.text = labelTXT + "\n" + units;
        gaugeImage.color = redColor;

        setsafeZoneLinePos();
        setsafeZoneLineY();
        setGreenAreaX();
        setGreenAreaY();
    }
    private void setsafeZoneLinePos()
    {
        //set verticle position of safeZoneLine to where the safeZoneVal is
        float normalizedsafePosition = ((safeZoneValue - minValue) / (maxValue - minValue));
        float gaugeWidth = gaugeImage.rectTransform.rect.width;
        xPosition = Mathf.Lerp(-gaugeWidth / 2f, gaugeWidth / 2f, normalizedsafePosition);
        safeZoneLine.localPosition = new Vector3(xPosition, safeZoneYPosition, 0f);
    }
    private void setsafeZoneLineY()
    {
        safeZoneLine.localScale = new Vector3(0.015f, 1f, 0f);
    }
    private void Update()
    {
        needlePosUpdate();
        updateValTxt();
    }

    void needlePosUpdate()
    {
        // Calculate the currentt vals and where the needle needs to be
        float normalizedPosition = ((currentValue - minValue) / (maxValue - minValue));
        float gaugeWidth = gaugeImage.rectTransform.rect.width;
        float targetXPosition = Mathf.Lerp(-gaugeWidth / 2f, gaugeWidth / 2f, normalizedPosition);

        // Set the position of the needle to the gauge val
        needle.localPosition = new Vector3(targetXPosition, needleYPosition, 0f);
    }
    void updateValTxt()
    {
        valueText.text = currentValue.ToString("F2") + units; // Update with appropriate units
    }

    private void setGreenAreaX()
    {
        // Calculate the normalized position, the width from RectTransform, width of GA from norm safe zone pos, centerpos
        float normalizedSafeZonePosition = (safeZoneValue - minValue) / (maxValue - minValue);
        float gaugeWidth = gaugeImage.rectTransform.rect.width;
        float greenAreaWidth = Mathf.Lerp(0f, gaugeWidth, normalizedSafeZonePosition);
        float centerXPosition = Mathf.Lerp(-gaugeWidth / 2f, gaugeWidth / 2f, normalizedSafeZonePosition / 2f);

        // Set the scale of the green area
        Vector2 newScale = greenArea.localScale;
        newScale.x = greenAreaWidth / gaugeWidth; 
        greenArea.localScale = newScale;

        // Set the position of the green area
        Vector3 newPosition = greenArea.localPosition;
        newPosition.x = centerXPosition;
        greenArea.localPosition = newPosition;
    }
    private void setGreenAreaY()
    {
        greenArea.localScale = new Vector3(greenArea.localScale.x, 1f, 0f);
    }
}
