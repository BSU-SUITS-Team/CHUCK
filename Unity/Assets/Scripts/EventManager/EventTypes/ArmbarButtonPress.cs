namespace ARSIS.EventManager
{
    [System.Serializable]
    public class ArmbarButtonPressData
    {
        public string action { get; set; }
        public int button { get; set; }
        public string key { get; set; }
        public string target { get; set; }
        public string source { get; set; }
    }

    [System.Serializable]
    public class ArmbarButtonPress : BaseArsisEvent
    {
        public ArmbarButtonPressData data { get; set; }

        public override string ToString()
        {
            return $"Armbar button {data?.button} press event";
        }
    }
}
