namespace ARSIS.EventManager
{
    [System.Serializable]
    public class AiaMessageData
    {
        public string message { get; set; }
        public string source { get; set; }
        public string target { get; set; }
    }

    [System.Serializable]
    public class AiaMessage : BaseArsisEvent
    {
        public AiaMessageData data { get; set; }

        public override string ToString()
        {
            return "AIA message event";
        }
    }
}
