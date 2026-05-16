namespace ARSIS.EventManager
{
    [System.Serializable]
    public class VoiceTranscriptionCommandData
    {
        public string action { get; set; }
        public string target { get; set; }
        public string source { get; set; }
        public bool transcribing { get; set; }
    }

    [System.Serializable]
    public class VoiceTranscriptionCommand : BaseArsisEvent
    {
        public VoiceTranscriptionCommandData data { get; set; }

        public override string ToString()
        {
            return "Voice transcription command event";
        }
    }
}
