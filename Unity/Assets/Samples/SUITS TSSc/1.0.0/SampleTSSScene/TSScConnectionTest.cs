using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;

using TMPro;

public class TSScConnectionTest : MonoBehaviour
{
    // Connection
    string TSShost;
    string port;
    string url;
    //int    team_number;
    bool   connected;
    float  time_since_last_update;

    // Database Jsons
    //bool UIAUpdated;
   // string UIAJsonString;
    //bool DCUUpdated;
   // string DCUJsonString;
    bool ROVERUpdated;
    string ROVERJsonString;
    bool EVAUpdated;
    string EVAJsonString;
    bool LTVUpdated;
    string LTVJsonString;
    bool LTVErrorUpdated;
    string LTVErrorJsonString;

    // bool SPECUpdated;
    // string SPECJsonString;
    // bool TELEMETRYUpdated;
    // string TELEMETRYJsonString;
    // bool COMMUpdated;
    // string COMMJsonString;
    // bool IMUUpdated;
    // string IMUJsonString;

    // Connect to TSSc with a specific team number
    public void ConnectToTSSHost(string TSShost) //, int team_number)
    {
        this.TSShost = TSShost;
        this.port = "14141";
        //this.team_number = team_number;
        this.url = "http://" + this.TSShost + ":" + this.port;
        Debug.Log(this.url);

        // Test Connection
        StartCoroutine(GetRequest(this.url));
    }

    public void DisconnectFromHost()
    {
        this.connected = false;
    }

    // This Function is called when the program begins
    void Start()
    {
        this.connected = false;
    }

    // This Function is called each render frame
    void Update()
    {
        // If you are connected to TSSc
        if (this.connected)
        {
            // Each Second
            time_since_last_update += Time.deltaTime;
            if (time_since_last_update > 1.0f)
            {
                // Pull TSSc Updates
                StartCoroutine(GetEVAState());
                StartCoroutine(GetROVERState()); 
                StartCoroutine(GetLTVState());
                StartCoroutine(GetLTVErrorState());
                // StartCoroutine(GetTELEMETRYState());
                // StartCoroutine(GetCOMMState());
                // StartCoroutine(GetIMUState());
                time_since_last_update = 0.0f;
            }
        }
    }

    IEnumerator GetRequest(string uri)
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get(uri))
        {
            yield return webRequest.SendWebRequest();
            string[] pages = uri.Split('/');
            int page = pages.Length - 1;

            switch (webRequest.result)
            {
                case UnityWebRequest.Result.ConnectionError:
                case UnityWebRequest.Result.DataProcessingError:
                    Debug.LogError(pages[page] + ": Error: " + webRequest.error);
                    break;
                case UnityWebRequest.Result.ProtocolError:
                    Debug.LogError(pages[page] + ": HTTP Error: " + webRequest.error);
                    break;
                case UnityWebRequest.Result.Success:
                    Debug.Log(pages[page] + ":\nReceived: " + webRequest.downloadHandler.text);
                    this.connected = true;
                    break;
            }

        }
    }

    ///////////////////////////////////////////// UIA
    
    //  IEnumerator GetEVAState()
    // {
    //     using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/data/EVA.json"))
    //     {
    //         // Request and wait for the desired page.
    //         yield return webRequest.SendWebRequest();

    //         switch (webRequest.result)
    //         {
    //             case UnityWebRequest.Result.Success:
    //                 if (this.EVAJsonString != webRequest.downloadHandler.text)
    //                 {
    //                     this.EVAupdated = true;
    //                     this.EVAJsonString = webRequest.downloadHandler.text;
    //                 }
    //                 break;
    //         }

    //     }
    // }

    // IEnumerator GetUIAState()
    // {
    //     using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/data/EVA.json"))
    //     {
    //         // Request and wait for the desired page.
    //         yield return webRequest.SendWebRequest();

    //         switch (webRequest.result)
    //         {
    //             case UnityWebRequest.Result.Success:
    //                 if (this.UIAJsonString != webRequest.downloadHandler.text)
    //                 {
    //                     this.UIAUpdated = true;
    //                     this.UIAJsonString = webRequest.downloadHandler.text;
    //                 }
    //                 break;
    //         }

    //     }
    // }

    // public string GetUIAJsonString()
    // {
    //     UIAUpdated = false;
    //     return this.UIAJsonString;
    // }

    // public bool isUIAUpdated()
    // {
    //     return UIAUpdated;
    // }

    // ///////////////////////////////////////////// DCU

    // IEnumerator GetDCUState()
    // {
    //     using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/json_data/DCU.json"))
    //     {
    //         // Request and wait for the desired page.
    //         yield return webRequest.SendWebRequest();

    //         switch (webRequest.result)
    //         {
    //             case UnityWebRequest.Result.Success:
    //                 if (this.DCUJsonString != webRequest.downloadHandler.text)
    //                 {
    //                     this.DCUUpdated = true;
    //                     this.DCUJsonString = webRequest.downloadHandler.text;
    //                     Debug.Log(this.DCUJsonString);
    //                 }
    //                 break;
    //         }

    //     }
    // }

    // public string GetDCUJsonString()
    // {
    //     DCUUpdated = false;
    //     return this.DCUJsonString;
    // }

    // public bool isDCUUpdated()
    // {
    //     return DCUUpdated;
    // }

    ///////////////////////////////////////////// ROVER

    IEnumerator GetROVERState()
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/data/ROVER.json"))
        {
            // Request and wait for the desired page.
            yield return webRequest.SendWebRequest();

            switch (webRequest.result)
            {
                case UnityWebRequest.Result.Success:
                    if (this.ROVERJsonString != webRequest.downloadHandler.text)
                    {
                        this.ROVERUpdated = true;
                        this.ROVERJsonString = webRequest.downloadHandler.text;
                        Debug.Log(this.ROVERJsonString);
                    }
                    break;
            }

        }
    }

    public string GetROVERJsonString()
    {
        ROVERUpdated = false;
        return this.ROVERJsonString;
    }

    public bool isROVERUpdated()
    {
        return ROVERUpdated;
    }

    IEnumerator GetEVAState()
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/data/EVA.json"))
        {
            // Request and wait for the desired page.
            yield return webRequest.SendWebRequest();

            switch (webRequest.result)
            {
                case UnityWebRequest.Result.Success:
                    if (this.EVAJsonString != webRequest.downloadHandler.text)
                    {
                        this.EVAUpdated = true;
                        this.EVAJsonString = webRequest.downloadHandler.text;
                        Debug.Log(this.EVAJsonString);
                    }
                    break;
            }

        }
    }

    public string GetEVAJsonString()
    {
        EVAUpdated = false;
        return this.EVAJsonString;
    }

    public bool isEVAUpdated()
    {
        return EVAUpdated;
    }

    IEnumerator GetLTVState()
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/data/LTV.json"))
        {
            // Request and wait for the desired page.
            yield return webRequest.SendWebRequest();

            switch (webRequest.result)
            {
                case UnityWebRequest.Result.Success:
                    if (this.LTVJsonString != webRequest.downloadHandler.text)
                    {
                        this.LTVUpdated = true;
                        this.LTVJsonString = webRequest.downloadHandler.text;
                        Debug.Log(this.LTVJsonString);
                    }
                    break;
            }

        }
    }

    public string GetLTVJsonString()
    {
        LTVUpdated = false;
        return this.LTVJsonString;
    }

    public bool isLTVUpdated()
    {
        return LTVUpdated;
    }

        IEnumerator GetLTVErrorState()
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/data/LTV_ERRORS.json"))
        {
            // Request and wait for the desired page.
            yield return webRequest.SendWebRequest();

            switch (webRequest.result)
            {
                case UnityWebRequest.Result.Success:
                    if (this.LTVErrorJsonString != webRequest.downloadHandler.text)
                    {
                        this.LTVErrorUpdated = true;
                        this.LTVErrorJsonString = webRequest.downloadHandler.text;
                        Debug.Log(this.LTVErrorJsonString);
                    }
                    break;
            }

        }
    }

    public string GetLTVErrorJsonString()
    {
        LTVErrorUpdated = false;
        return this.LTVErrorJsonString;
    }

    public bool isLTVErrorUpdated()
    {
        return LTVErrorUpdated;
    }

    ///////////////////////////////////////////// SPEC

    // IEnumerator GetSPECState()
    // {
    //     using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/json_data/SPEC.json"))
    //     {
    //         // Request and wait for the desired page.
    //         yield return webRequest.SendWebRequest();

    //         switch (webRequest.result)
    //         {
    //             case UnityWebRequest.Result.Success:
    //                 if (this.SPECJsonString != webRequest.downloadHandler.text)
    //                 {
    //                     this.SPECUpdated = true;
    //                     this.SPECJsonString = webRequest.downloadHandler.text;
    //                     Debug.Log(this.SPECJsonString);
    //                 }
    //                 break;
    //         }

    //     }
    // }

    // public string GetSPECJsonString()
    // {
    //     SPECUpdated = false;
    //     return this.SPECJsonString;
    // }

    // public bool isSPECUpdated()
    // {
    //     return SPECUpdated;
    // }

    ///////////////////////////////////////////// TELEMETRY

    // IEnumerator GetTELEMETRYState()
    // {
    //     using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/json_data/teams/" + this.team_number + "/TELEMETRY.json"))
    //     {
    //         // Request and wait for the desired page.
    //         yield return webRequest.SendWebRequest();

    //         switch (webRequest.result)
    //         {
    //             case UnityWebRequest.Result.Success:
    //                 if (this.TELEMETRYJsonString != webRequest.downloadHandler.text)
    //                 {
    //                     this.TELEMETRYUpdated = true;
    //                     this.TELEMETRYJsonString = webRequest.downloadHandler.text;
    //                     Debug.Log(this.TELEMETRYJsonString);
    //                 }
    //                 break;
    //         }

    //     }
    // }

    // public string GetTELEMETRYJsonString()
    // {
    //     TELEMETRYUpdated = false;
    //     return this.TELEMETRYJsonString;
    // }

    // public bool isTELEMETRYUpdated()
    // {
    //     return TELEMETRYUpdated;
    // }

    // ///////////////////////////////////////////// COMM

    // IEnumerator GetCOMMState()
    // {
    //     using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/json_data/COMM.json"))
    //     {
    //         // Request and wait for the desired page.
    //         yield return webRequest.SendWebRequest();

    //         switch (webRequest.result)
    //         {
    //             case UnityWebRequest.Result.Success:
    //                 if (this.COMMJsonString != webRequest.downloadHandler.text)
    //                 {
    //                     this.COMMUpdated = true;
    //                     this.COMMJsonString = webRequest.downloadHandler.text;
    //                     Debug.Log(this.COMMJsonString);
    //                 }
    //                 break;
    //         }

    //     }
    // }

    // public string GetCOMMJsonString()
    // {
    //     COMMUpdated = false;
    //     return this.COMMJsonString;
    // }

    // public bool isCOMMUpdated()
    // {
    //     return COMMUpdated;
    // }

    // ///////////////////////////////////////////// IMU

    // IEnumerator GetIMUState()
    // {
    //     using (UnityWebRequest webRequest = UnityWebRequest.Get(this.url + "/json_data/IMU.json"))
    //     {
    //         // Request and wait for the desired page.
    //         yield return webRequest.SendWebRequest();

    //         switch (webRequest.result)
    //         {
    //             case UnityWebRequest.Result.Success:
    //                 if (this.IMUJsonString != webRequest.downloadHandler.text)
    //                 {
    //                     this.IMUUpdated = true;
    //                     this.IMUJsonString = webRequest.downloadHandler.text;
    //                     Debug.Log(this.IMUJsonString);
    //                 }
    //                 break;
    //         }

    //     }
    // }

    // public string GetIMUJsonString()
    // {
    //     IMUUpdated = false;
    //     return this.IMUJsonString;
    // }

    // public bool isIMUUpdated()
    // {
    //     return IMUUpdated;
    // }




}
