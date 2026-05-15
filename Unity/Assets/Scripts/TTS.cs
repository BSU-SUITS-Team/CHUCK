using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Audio;

public class TTS : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        // Assuming this is part of a script attached to your object
        TextToSpeech tts = GetComponent<TextToSpeech>();
        tts.StartSpeaking("Hello World");
    }
}
