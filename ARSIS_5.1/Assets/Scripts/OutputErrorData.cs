using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class OutputErrorData : MonoBehaviour {

    public Text m_ErrorText;

    public float m_LastSirenTime;

    [Header("Audio")]
    public AudioSource m_Source;

    public AudioClip m_bad;

    public void Start()
    {
        m_LastSirenTime = -30f; //so first error text can trigger alarm
    }

    public void OutputErrorText(string s)
    {
        if (Time.realtimeSinceStartup - m_LastSirenTime >= 30f)
        {
            m_LastSirenTime = Time.realtimeSinceStartup;

            m_ErrorText.text += s + "\n";
            m_Source.clip = m_bad;
            m_Source.loop = false;
            m_Source.Play();
        }

    }

    public void ClearText()
    {
        m_ErrorText.text = "";
        m_Source.loop = false; 
        m_Source.Stop(); 
    }
}