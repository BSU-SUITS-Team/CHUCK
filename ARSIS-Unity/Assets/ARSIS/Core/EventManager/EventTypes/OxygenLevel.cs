using ARSISEventSystem;

public class OxygenLevel : BaseArsisEvent
{
    public readonly float value;

    public OxygenLevel(float value)
    {
        this.value = value;
    }

    public static implicit operator float(OxygenLevel e)
    {
        return e.value;
    }
}
