namespace ARSIS.EventManager
{
    [System.Serializable]
    public class HololensCommandData
    {
        public string action { get; set; }
        public string window { get; set; }
        public string procedure { get; set; }
        public string target { get; set; }
        public string source { get; set; }
    }

    [System.Serializable]
    public class HololensCommand : BaseArsisEvent
    {
        public HololensCommandData data { get; set; }

        public override string ToString()
        {
            return "Hololens command event";
        }
    }
}
