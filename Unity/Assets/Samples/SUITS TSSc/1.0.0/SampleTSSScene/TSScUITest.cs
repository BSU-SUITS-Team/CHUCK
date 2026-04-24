using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using TMPro;

public class TSScUITest : MonoBehaviour
{
    // TSSc Connection
    public TSScConnectionTest TSSc2;

    // UI Input
    // public TMP_InputField InputFieldUrl;
    public InputField InputFieldUrl2;
    // public Button     ConnectButton2;
    public GameObject ConnectButton2;

    // // UI Display
    // public TMP_Text UIAJsonDisplay;
    // public TMP_Text DCUJsonDisplay;
    public TMP_Text ROVERJsonDisplay2;
    public TMP_Text EVAJsonDisplay2;
    public TMP_Text LTVJsonDisplay2; 
    public TMP_Text LTVErrorJsonDisplay2;
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
        if (TSSc2.isROVERUpdated())
        {
            Debug.Log("ROVER Updated");

            // Get the Updated ROVER Json
            string ROVERJsonString = TSSc2.GetROVERJsonString();

            // Displays the ROVER data to the screen (Your's should be more complicated)
            ROVERJsonDisplay2.text = ROVERJsonString;
        }

        // Check if the EVA data has been updated
        if (TSSc2.isEVAUpdated())
        {
            Debug.Log("EVA Updated");

            // Get the Updated EVA Json
            string EVAJsonString = TSSc2.GetEVAJsonString();

            // Displays the EVA data to the screen (Your's should be more complicated)
            EVAJsonDisplay2 .text = EVAJsonString;
        }

        // Check if the LTV data has been updated
        if (TSSc2.isLTVUpdated())
        {
            Debug.Log("LTV Updated");

            // Get the Updated LTV Json
            string LTVJsonString = TSSc2.GetLTVJsonString();

            // Displays the LTV data to the screen (Your's should be more complicated)
            LTVJsonDisplay2.text = LTVJsonString;
        }

        // Check if the LTV Error data has been updated
        if (TSSc2.isLTVErrorUpdated())
        {
            Debug.Log("LTV Error Updated");

            // Get the Updated LTV Error Json
            string LTVErrorJsonString = TSSc2.GetLTVErrorJsonString();

            // Displays the LTV Error data to the screen (Your's should be more complicated)
            LTVErrorJsonDisplay2.text = LTVErrorJsonString;
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
        string TSShost = InputFieldUrl2.text;

        // Print Hostname to Logs
        Debug.Log("Button Pressed: " + TSShost);

        // Connect to TSSc at that Host
        TSSc2.ConnectToTSSHost(TSShost);
    }

    public void Disconnect_Button()
    {
        // Disconnects from TSS when
        TSSc2.DisconnectFromHost();
    }

}
