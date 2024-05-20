using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class archGauge : MonoBehaviour
{
    public RectTransform needle;
    public RectTransform greenAreaRect;
    public Image biosGaugeImage;
    public Image GreenAreaImage;
    public Text biosValueText;
    public Text labelText;

    public string labelTXT;
    public string units;

    public float biosValue = 0;
    public float minValue = 0;
    public float maxValue = 100;

    public float lowerBounds = 30;
    public float upperBounds = 70;
    public float nominalVal = 50;

    void Start()
    {
        //SetInitialGaugeColors();

        labelText.text = labelTXT + "\n" + units;
        SetGaugeNumbers();
        SetGreenArea();
    }
    void SetGaugeNumbers()
    {
        SetNumberPosition(-90f, 90f); // Max value at 90 degrees, rotation -90 degrees
        SetNumberPosition(90f, -90f); // Min value at -90 degrees, rotation 90 degrees
        SetNumberPosition(0f, 0f); // Nominal value at 0 degrees, rotation 0 degrees

        // Setting upper and lower bounds
        //float lowerBoundAngle = -45f + ((nominalVal - minValue) / (maxValue - minValue)) * 90f;
        //float upperBoundAngle = 45f + ((nominalVal - minValue) / (maxValue - minValue)) * 90f;
        //SetNumberPosition(lowerBoundAngle, -45f);
        //SetNumberPosition(upperBoundAngle, 45f);
    }

    void SetNumberPosition(float angle, float rotation)
    {
        // Get the width of the gauge background or another appropriate object
        float semiMajorAxis = biosGaugeImage.rectTransform.sizeDelta.x / 2f;

        // Calculate the radius of the gauge arc based on the semi-major axis
        float radius = semiMajorAxis;

        // Calculate the position of the number along the gauge arc using trigonometry
        float xPos = radius * Mathf.Cos(Mathf.Deg2Rad * angle);
        float yPos = radius * Mathf.Sin(Mathf.Deg2Rad * angle);
        Vector3 newPosition = new Vector3(xPos, yPos, 0f);
    }


    void SetGreenArea()
    {
        float greenAreaRot = Mathf.Abs(lowerBounds - minValue)/maxValue*-180;
        //|start - end|/high = %
        // * -180 degrees (half circle)
        greenAreaRect.rotation = Quaternion.Euler (0,0, greenAreaRot);
        Debug.Log(greenAreaRot);

        // float greenAreaFill = (Mathf.Abs(minValue - (upperBounds-lowerBounds))/maxValue)/2;
        //float greenAreaFill = Mathf.Abs((lowerBounds - upperBounds) / (maxValue - lowerBounds) / 2);

        float greenAreaFill = Mathf.Abs(upperBounds - lowerBounds) / maxValue / 2;
        // 1 fill is full circle, 0.5 or 1/2 fill is half circle
        GreenAreaImage.fillAmount = greenAreaFill;
        Debug.Log(greenAreaFill);

    }
    
    void Update()
    {
        biosChange(biosValue);
    }

    void biosChange(float biosValue)
    {
        /* 
        Ensure bios value stays within bounds
            biosValue = Mathf.Clamp(biosValue, minValue, maxValue);
        either call another function here or set the biosValue so it uses the incomming data instead
        of having to set it in the inspector or here in the code.
        */

        float normalizedPosition = (biosValue - minValue) / (maxValue - minValue);
        float minValueAngle = 90f;
        float maxValueAngle = -90f;
        float lowerValAngle = -45f;
        float upperValAngle = 45f;
        float lowerInterpolatedAngle = Mathf.Lerp(minValueAngle, lowerValAngle, normalizedPosition);
        float upperInterpolatedAngle = Mathf.Lerp(upperValAngle, maxValueAngle, normalizedPosition);
        float adjustedNeedleAngle = Mathf.Lerp(lowerInterpolatedAngle, upperInterpolatedAngle, normalizedPosition);

        needle.localEulerAngles = new Vector3(0, 0, adjustedNeedleAngle);
        biosValueText.text = biosValue.ToString("F2") + units;
    }
}
