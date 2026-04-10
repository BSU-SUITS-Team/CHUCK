using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using TMPro;

public class TSScUI : MonoBehaviour
{
    // TSSc Connection
    public TSScConnection TSSc;

    // UI Input
    public TMP_InputField InputFieldUrl;
    public Button     ConnectButton;

    // // UI Display
    // public TMP_Text UIAJsonDisplay;
    // public TMP_Text DCUJsonDisplay;
    public TMP_Text ROVERJsonDisplay;
    public TMP_Text EVAJsonDisplay;
    public TMP_Text LTVJsonDisplay; 
    public TMP_Text LTVErrorJsonDisplay;
    // public TMP_Text SPECJsonDisplay;
    // public TMP_Text TELEMETRYJsonDisplay;
    // public TMP_Text COMMJsonDisplay;
    // public TMP_Text IMUJsonDisplay;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // // Check if the UIA data has been updated
        // if (TSSc.isUIAUpdated())
        // {
        //     Debug.Log("UIA Updated");
        //     string UIAJsonString = TSSc.GetUIAJsonString();

        //     // Displays the data on the screen (Your's should be more complicated)
        //     UIAJsonDisplay.text = UIAJsonString;
        // }

        // // Check if the DCU data has been updated
        // if (TSSc.isDCUUpdated())
        // {
        //     Debug.Log("DCU Updated");

        //     // Get the Updated DCU Json
        //     string DCUJsonString = TSSc.GetDCUJsonString();

        //     // Displays the DCU data to the screen (Your's should be more complicated)
        //     DCUJsonDisplay.text = DCUJsonString;
        // }

        // Check if the ROVER data has been updated
        if (TSSc.isROVERUpdated())
        {
            Debug.Log("ROVER Updated");

            // Get the Updated ROVER Json
            string ROVERJsonString = TSSc.GetROVERJsonString();

            // Displays the ROVER data to the screen (Your's should be more complicated)
            ROVERJsonDisplay.text = ROVERJsonString;
        }

        // Check if the EVA data has been updated
        if (TSSc.isEVAUpdated())
        {
            Debug.Log("EVA Updated");

            // Get the Updated EVA Json
            string EVAJsonString = TSSc.GetEVAJsonString();

            // Displays the EVA data to the screen (Your's should be more complicated)
            EVAJsonDisplay.text = EVAJsonString;
        }

        // Check if the LTV data has been updated
        if (TSSc.isLTVUpdated())
        {
            Debug.Log("LTV Updated");

            // Get the Updated LTV Json
            string LTVJsonString = TSSc.GetLTVJsonString();

            // Displays the LTV data to the screen (Your's should be more complicated)
            LTVJsonDisplay.text = LTVJsonString;
        }

        // Check if the LTV Error data has been updated
        if (TSSc.isLTVErrorUpdated())
        {
            Debug.Log("LTV Error Updated");

            // Get the Updated LTV Error Json
            string LTVErrorJsonString = TSSc.GetLTVErrorJsonString();

            // Displays the LTV Error data to the screen (Your's should be more complicated)
            LTVErrorJsonDisplay.text = LTVErrorJsonString;
        }

        // // Check if the IMU data has been updated
        // if (TSSc.isIMUUpdated())
        // {
        //     Debug.Log("IMU Updated");

        //     // Get the Updated IMU Json
        //     string IMUJsonString = TSSc.GetIMUJsonString();

        //     // Displays the IMU data to the screen (Your's should be more complicated)
        //     IMUJsonDisplay.text = IMUJsonString;
        // }
    }

    // On Connect Button Press
    public void Connect_Button()
    {
        // Get URL in Text Field
        string host = InputFieldUrl.text;

        // Print Hostname to Logs
        Debug.Log("Button Pressed: " + host);

        // Connect to TSSc at that Host
        TSSc.ConnectToHost(host);
    }

    public void Disconnect_Button()
    {
        // Disconnects from TSS when
        TSSc.DisconnectFromHost();
    }

}
