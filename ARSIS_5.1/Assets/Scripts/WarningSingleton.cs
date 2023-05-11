using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro; 

public class WarningSingleton : MonoBehaviour
{
    public static WarningSingleton m_Singleton;

    public GameObject warningPrefab;
    public GameObject warningGrid;
    public Dictionary<string, GameObject> warnings; 
    
    [HideInInspector]
    public bool m_DataInWarning = false;

    private List<string> m_Warnings;
    // Start is called before the first frame update
    void Awake()
    {
        m_Singleton = this;
        m_Warnings = new List<string>();
        warnings = new Dictionary<string, GameObject>(); 
    }

    public void BiometricInWarning(string s)
    {
        if (m_Warnings.Contains(s)) return;

        FindObjectOfType<OutputErrorData>().OutputErrorText("Warning in Biometrics"); // this plays the sound
        string error = "Biometric out of range"; 
        if (s == "Fan Tachometer") // Fan Error 
        {
            error = "Fan error";
            if (warnings.ContainsKey(error)) return;
            ErrorManager.S.fanError(); 
        } else if (s == "Time Life Oxygen" || s == "Oxygen Pressure" || s == "Oxygen Rate") // Oxygen Error
        {
            error = "Oxygen error";
            if (warnings.ContainsKey(error)) return;
            ErrorManager.S.O2Error(); 
        } else if (s == "SUB Temperature") // Temperature error
        {
            error = "Temperature error";
            ErrorManager.S.TemperatureError();
            if (warnings.ContainsKey(error)) return;
        } else if (s == "SUB Pressure" || s == "Internal Suit Pressure" || s == "H20 Gas Pressure" || s == "H20 Liquid Pressure" || s == "SOP Pressure")
        {
            error = "Pressure error";
            if (warnings.ContainsKey(error)) return;
            ErrorManager.S.PressureError(); 
        }
        if (error == "Biometric out of range")
        {
            m_DataInWarning = true;
            m_Warnings.Add(s);
            return;
        } 
        m_Warnings.Add(error);
        //if (warnings.ContainsKey(error)) return; 
        GameObject warningGO = Instantiate(warningPrefab);
        TextMeshProUGUI warningText = warningGO.GetComponentInChildren<TextMeshProUGUI>();
        warningText.text = error; 
        warningGO.transform.SetParent(warningGrid.transform, false);
        warnings[error] = warningGO; 
    }

    public void BiometricInNominal(string s)
    {
        string error = "Biometric out of range";
        if (s == "Fan Tachometer") // Fan Error 
        {
            error = "Fan error";
        }
        else if (s == "Time Life Oxygen" || s == "Oxygen Pressure" || s == "Oxygen Rate") // Oxygen Error
        {
            error = "Oxygen error";
        }
        else if (s == "SUB Temperature") // Temperature error
        {
            error = "Temperature error";
        }
        else if (s == "SUB Pressure" || s == "Internal Suit Pressure" || s == "H20 Gas Pressure" || s == "H20 Liquid Pressure" || s == "SOP Pressure")
        {
            error = "Pressure error";
        }
        if (error == "Biometric out of range") return; 
        if (m_Warnings.Contains(error))
        {
            m_Warnings.Remove(error);
            if (warnings.ContainsKey(error))
            {
                Destroy(warnings[error]);
                warnings.Remove(error); 
            }
        }

        if (m_Warnings.Count == 0)
        {
            m_DataInWarning = false;
        }

    }
}
