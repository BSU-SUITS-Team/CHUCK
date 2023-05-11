using Newtonsoft.Json;
using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.Networking;

public class CustomVisionAnalyser : MonoBehaviour
{
    /// Unique instance of this class
    public static CustomVisionAnalyser Instance;

    /// Insert your prediction key here
    private string predictionKey = "- 98d0b5c5bbba45bbb78791098b9f0fa1 -";

    /// Insert your prediction endpoint here
    private string predictionEndpoint = "https://arsis4.cognitiveservices.azure.com/customvision/v3.0/Prediction/97de2dea-a0aa-45bb-9dc3-cdc0922011d8/detect/iterations/CupTest1/image";

    /// Bite array of the image to submit for analysis
    [HideInInspector] public byte[] imageBytes;

    //Initialize this class
    private void Awake()
    {
        // Allows this instance to behave like a singleton
        Instance = this;
    }

    /// Call the Computer Vision Service to submit the image.
    public IEnumerator AnalyseLastImageCaptured(string imagePath)
    {
        Debug.Log("Analyzing...");

        WWWForm webForm = new WWWForm();

        using (UnityWebRequest unityWebRequest = UnityWebRequest.Post(predictionEndpoint, webForm))
        {
            // Gets a byte array out of the saved image
            imageBytes = GetImageAsByteArray(imagePath);

            unityWebRequest.SetRequestHeader("Content-Type", "application/octet-stream");
            unityWebRequest.SetRequestHeader("Prediction-Key", predictionKey);

            // The upload handler will help uploading the byte array with the request
            unityWebRequest.uploadHandler = new UploadHandlerRaw(imageBytes);
            unityWebRequest.uploadHandler.contentType = "application/octet-stream";

            // The download handler will help receiving the analysis from Azure
            unityWebRequest.downloadHandler = new DownloadHandlerBuffer();

            // Send the request
            yield return unityWebRequest.SendWebRequest();

            string jsonResponse = unityWebRequest.downloadHandler.text;

            Debug.Log("response: " + jsonResponse);

            // Create a texture. Texture size does not matter, since
            // LoadImage will replace with the incoming image size.
            Texture2D tex = new Texture2D(1, 1);
            tex.LoadImage(imageBytes);
           SceneOrganiser.Instance.quadRenderer.material.SetTexture("_MainTex", tex);

            // The response will be in JSON format, therefore it needs to be deserialized
            AnalysisRootObject analysisRootObject = new AnalysisRootObject();
            analysisRootObject = JsonConvert.DeserializeObject<AnalysisRootObject>(jsonResponse);

            SceneOrganiser.Instance.FinaliseLabel(analysisRootObject);
        }
    }

    /// Returns the contents of the specified image file as a byte array.
    static byte[] GetImageAsByteArray(string imageFilePath)
    {
        FileStream fileStream = new FileStream(imageFilePath, FileMode.Open, FileAccess.Read);

        BinaryReader binaryReader = new BinaryReader(fileStream);

        return binaryReader.ReadBytes((int)fileStream.Length);
    }
}
