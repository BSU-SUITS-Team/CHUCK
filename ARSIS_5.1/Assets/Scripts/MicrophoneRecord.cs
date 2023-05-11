using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(AudioSource))]

public class MicrophoneRecord : MonoBehaviour
{
    //A boolean that flags whether there's a connected microphone  
    private bool micConnected = false;

    //The maximum and minimum available recording frequencies  
    private int minFreq;
    private int maxFreq;

    private AudioSource goAudioSource;

    public static MicrophoneRecord S; 

    //Use this for initialization  
    void Start()
    {
        S = this; 
        //Check if there is at least one microphone connected  
        if (Microphone.devices.Length <= 0)
        {
            Debug.LogWarning("Microphone not connected!");
        }
        else //At least one microphone is present  
        {
            micConnected = true;

            //Get the default microphone recording capabilities  
            Microphone.GetDeviceCaps(null, out minFreq, out maxFreq);

            if (minFreq == 0 && maxFreq == 0)
            {
                //...meaning 44100 Hz can be used as the recording sampling rate  
                maxFreq = 44100;
            }

            goAudioSource = this.GetComponent<AudioSource>();
        }
    }

    public void startRecording()
    {
        if (micConnected)
        {
            //If the audio from any microphone isn't being captured  
            if (!Microphone.IsRecording(null))
            {
                goAudioSource.clip = Microphone.Start(null, true, 20, maxFreq);
            }

        } 
    }

    public AudioClip stopRecording()
    {
        if (micConnected)
        {
            Microphone.End(null); //Stop the audio recording  
            //goAudioSource.Play(); //Playback the recorded audio  

            return goAudioSource.clip; 
        }
        return null; 
    }

    public void playAudioClip(AudioClip clip)
    {
        if (!Microphone.IsRecording(null))
        {
            goAudioSource.clip = clip;
            goAudioSource.Play(); 
        }
    }

}
