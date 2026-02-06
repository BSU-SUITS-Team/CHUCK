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

    // UI Display
    public TMP_Text UIAJsonDisplay;
    public TMP_Text DCUJsonDisplay;
    public TMP_Text ROVERJsonDisplay;
    public TMP_Text SPECJsonDisplay;
    public TMP_Text TELEMETRYJsonDisplay;
    public TMP_Text COMMJsonDisplay;
    public TMP_Text IMUJsonDisplay;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Check if the UIA data has been updated
        if (TSSc.isUIAUpdated())
        {
            Debug.Log("UIA Updated");
            string UIAJsonString = TSSc.GetUIAJsonString();

            // Displays the data on the screen (Your's should be more complicated)
            UIAJsonDisplay.text = UIAJsonString;
        }

        // Check if the DCU data has been updated
        if (TSSc.isDCUUpdated())
        {
            Debug.Log("DCU Updated");

            // Get the Updated DCU Json
            string DCUJsonString = TSSc.GetDCUJsonString();

            // Displays the DCU data to the screen (Your's should be more complicated)
            DCUJsonDisplay.text = DCUJsonString;
        }

        // Check if the ROVER data has been updated
        if (TSSc.isROVERUpdated())
        {
            Debug.Log("ROVER Updated");

            // Get the Updated ROVER Json
            string ROVERJsonString = TSSc.GetROVERJsonString();

            // Displays the ROVER data to the screen (Your's should be more complicated)
            ROVERJsonDisplay.text = ROVERJsonString;
        }

        // Check if the SPEC data has been updated
        if (TSSc.isSPECUpdated())
        {
            Debug.Log("SPEC Updated");

            // Get the Updated SPEC Json
            string SPECJsonString = TSSc.GetSPECJsonString();

            // Displays the SPEC data to the screen (Your's should be more complicated)
            SPECJsonDisplay.text = SPECJsonString;
        }

        // Check if the TELEMETRY data has been updated
        if (TSSc.isTELEMETRYUpdated())
        {
            Debug.Log("TELEMETRY Updated");

            // Get the Updated TELEMETRY Json
            string TELEMETRYJsonString = TSSc.GetTELEMETRYJsonString();

            // Displays the TELEMETRY data to the screen (Your's should be more complicated)
            TELEMETRYJsonDisplay.text = TELEMETRYJsonString;
        }

        // Check if the COMM data has been updated
        if (TSSc.isCOMMUpdated())
        {
            Debug.Log("COMM Updated");

            // Get the Updated COMM Json
            string COMMJsonString = TSSc.GetCOMMJsonString();

            // Displays the COMM data to the screen (Your's should be more complicated)
            COMMJsonDisplay.text = COMMJsonString;
        }

        // Check if the COMM data has been updated
        if (TSSc.isIMUUpdated())
        {
            Debug.Log("IMU Updated");

            // Get the Updated IMU Json
            string IMUJsonString = TSSc.GetIMUJsonString();

            // Displays the IMU data to the screen (Your's should be more complicated)
            IMUJsonDisplay.text = IMUJsonString;
        }
    }

    // On Connect Button Press
    public void Connect_Button()
    {
        // Get URL in Text Field
        string host = InputFieldUrl.text;

        // Print Hostname to Logs
        Debug.Log("Button Pressed: " + host);

        // Connect to TSSc at that Host
        TSSc.ConnectToHost(host, 0);
    }

    public void Disconnect_Button()
    {
        // Disconnects from TSS when
        TSSc.DisconnectFromHost();
    }

}
