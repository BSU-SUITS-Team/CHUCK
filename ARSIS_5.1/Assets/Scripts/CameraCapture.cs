using System.Collections;
using System.Collections.Generic;
using UnityEngine.Windows.WebCam; 
using UnityEngine;
using UnityEngine.UI; 

// https://docs.microsoft.com/en-us/windows/mixed-reality/develop/unity/locatable-camera-in-unity
// https://github.com/deancohen1234/BSU-NASA-SUITS/blob/master/ARSIS/Assets/Scripts/HoloLensSnapshotTest.cs

public class CameraCapture : MonoBehaviour
{
    public static CameraCapture S;
    public RawImage image; 

    private PhotoCapture photoCaptureObject = null;


    void Start()
    {
        S = this; 
    }

    public void Capture()
    {
        PhotoCapture.CreateAsync(false, OnPhotoCaptureCreated);
    }
    
    void OnPhotoCaptureCreated(PhotoCapture captureObject)
    {
        photoCaptureObject = captureObject;

        List<Resolution> resolutions = new List<Resolution>(PhotoCapture.SupportedResolutions);
        Resolution cameraResolution = resolutions[0]; 

        CameraParameters c = new CameraParameters();
        c.hologramOpacity = 0.0f;
        c.cameraResolutionWidth = cameraResolution.width;
        c.cameraResolutionHeight = cameraResolution.height;
        c.pixelFormat = CapturePixelFormat.BGRA32;

        captureObject.StartPhotoModeAsync(c, OnPhotoModeStarted);
    }

    void OnStoppedPhotoMode(PhotoCapture.PhotoCaptureResult result)
    {
        photoCaptureObject.Dispose();
        photoCaptureObject = null;
    }

    private void OnPhotoModeStarted(PhotoCapture.PhotoCaptureResult result)
    {
        if (result.success)
        {
            photoCaptureObject.TakePhotoAsync(OnCapturedPhotoToMemory);
        }
        else
        {
            Debug.LogError("Unable to start photo mode!");
        }
    }

    void OnCapturedPhotoToMemory(PhotoCapture.PhotoCaptureResult result, PhotoCaptureFrame photoCaptureFrame)
    {
        if (result.success)
        {
            // Create our Texture2D for use and set the correct resolution
            List<Resolution> resolutions = new List<Resolution>(PhotoCapture.SupportedResolutions);
            Resolution cameraResolution = resolutions[0]; 
            Texture2D targetTexture = new Texture2D(cameraResolution.width, cameraResolution.height);
            // Copy the raw image data into our target texture
            photoCaptureFrame.UploadImageDataToTexture(targetTexture);
            targetTexture.wrapMode = TextureWrapMode.Clamp;

            // Display the image 
            SetImage(targetTexture);

            // Do as we wish with the texture such as apply it to a material, etc.
        }
        // Clean up
        photoCaptureObject.StopPhotoModeAsync(OnStoppedPhotoMode);
    }

    public void SetImage(Texture2D text)
    {
        image.texture = text;
        VoiceManager.S.captureEvent.Invoke();
    }

}
