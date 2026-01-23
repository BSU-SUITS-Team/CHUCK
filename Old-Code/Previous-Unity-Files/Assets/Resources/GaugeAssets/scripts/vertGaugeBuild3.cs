using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class vertGaugeBuild3 : MonoBehaviour
{
    //build-Up gauge green to red
    public Transform needle;
    public float minValue = 0f;
    public float maxValue = 100f;
    public float safeZoneValue = 50f;
    public float currentValue = 0f;

    private float needleYPosition = -20f;
    private float zoneYPosition = 0f;
    private float xPosition = 0f;

    public Text labelText; 
    public Text valueText;
    public string labelTXT;
    public string units;

    public Color greenColor = Color.green; 
    public Color redColor = Color.red;

    public RectTransform SafeLine;
    private Image gaugeImage; 

    void Start()
    {
        gaugeImage = GetComponent<Image>();
        labelText.text = labelTXT + "\n" + units;

        setSafeLine();
        setsafeZoneLineY();
    }

    private void setSafeLine()
    {
        float normalizedSafePosition =((safeZoneValue - minValue) / (maxValue - minValue));
        float gaugeWidth = gaugeImage.rectTransform.rect.width;
        xPosition = Mathf.Lerp(-gaugeWidth / 2f, gaugeWidth / 2f, normalizedSafePosition);
        SafeLine.localPosition = new Vector3(xPosition, zoneYPosition, 0f);
    }
    private void setsafeZoneLineY()
    {
        SafeLine.localScale = new Vector3(0.015f, 1f, 0f);
    }

    private void Update()
    {
        /*
        only allow needle to move between min and max values
        currentValue = Mathf.Clamp(currentValue, minValue, maxValue);

        use the below function to change the gauge value dynamically based on data stream
        updateBiosVal();
        */
        needlePosUpdate();
        updateValTxt();
        updateGaugeColor();
        setSafeLine();
    }

    void needlePosUpdate()
    {
        float normalizedPosition = ((currentValue - minValue) / (maxValue - minValue));
        float gaugeWidth = gaugeImage.rectTransform.rect.width;
        float targetXPosition = Mathf.Lerp(-gaugeWidth / 2f, gaugeWidth / 2f, normalizedPosition);
        needle.localPosition = new Vector3(targetXPosition, needleYPosition, 0f);
    }



    void updateValTxt()
    {
        valueText.text = currentValue.ToString("F2") + units; // Update with appropriate units
    }

    void updateGaugeColor()
    {
        // Change the color of the gauge based on the current value
        gaugeImage.color = currentValue <= safeZoneValue ? greenColor : redColor;
    }
}
