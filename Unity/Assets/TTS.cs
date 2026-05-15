using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Audio;

public class TTS : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        TextToSpeech tts = GetComponent<TextToSpeech>();
        if (tts != null)
        {
            tts.StartSpeaking("Hello, this is a test of the text to speech system.");
        }
        else
        {
            Debug.LogError("TextToSpeech component not found on this GameObject.");
        }
    }
}
