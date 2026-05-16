using System.Collections;
using System.Collections.Generic;
using ARSIS.EventManager;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Audio;

[RequireComponent(typeof(TextToSpeech))]
public class TTS : MonoBehaviour, IRenderable
{
    private const string AiaMessageEventType = "aia_message";

    private readonly object messageLock = new();

    private TextToSpeech textToSpeech;
    private long lastHandledEventTime;
    private long pendingEventTime;
    private string pendingMessage;
    private bool hasPendingMessage;

    void Awake()
    {
        textToSpeech = GetComponent<TextToSpeech>();
    }

    void OnEnable()
    {
        lastHandledEventTime = WebSocketClient.GetUnixTimeNanoseconds();
        EventDatastore.Instance.AddHandler(AiaMessageEventType, this);
    }

    void OnDisable()
    {
        EventDatastore.Instance.RemoveHandler(AiaMessageEventType, this);
    }

    public void Render(List<BaseArsisEvent> data)
    {
        AiaMessage latestMessage = null;
        long handledAfter;

        lock (messageLock)
        {
            handledAfter = lastHandledEventTime;
        }

        foreach (BaseArsisEvent baseEvent in data)
        {
            if (baseEvent is not AiaMessage aiaMessage)
                continue;

            if (aiaMessage.time <= handledAfter)
                continue;

            if (string.IsNullOrWhiteSpace(aiaMessage.data?.message))
                continue;

            if (latestMessage == null || aiaMessage.time > latestMessage.time)
                latestMessage = aiaMessage;
        }

        if (latestMessage == null)
            return;

        lock (messageLock)
        {
            if (latestMessage.time <= lastHandledEventTime)
                return;

            if (hasPendingMessage && pendingEventTime >= latestMessage.time)
                return;

            pendingEventTime = latestMessage.time;
            pendingMessage = latestMessage.data.message.Trim();
            hasPendingMessage = true;
        }
    }

    void Update()
    {
        string messageToSpeak = null;
        long eventTime = 0;

        lock (messageLock)
        {
            if (!hasPendingMessage)
                return;

            messageToSpeak = pendingMessage;
            eventTime = pendingEventTime;
            pendingMessage = null;
            hasPendingMessage = false;
            lastHandledEventTime = eventTime;
        }

        SpeakMessage(messageToSpeak);
    }

    private void SpeakMessage(string message)
    {
        if (string.IsNullOrWhiteSpace(message))
            return;

        if (textToSpeech == null)
            textToSpeech = GetComponent<TextToSpeech>();

        if (textToSpeech == null)
        {
            Debug.LogError("TTS: TextToSpeech component is missing.");
            return;
        }

        if (textToSpeech.IsSpeaking() || textToSpeech.SpeechTextInQueue())
            textToSpeech.StopSpeaking();

        textToSpeech.StartSpeaking(message);
    }
}
