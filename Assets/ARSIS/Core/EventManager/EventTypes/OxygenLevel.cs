using EventManagerSystem;

public class OxygenLevel : IArsisEvent
{
    private float value;
    
    public OxygenLevel(float value) {
        this.value = value;
    }

    public static implicit operator float(OxygenLevel e) {
        return e.value;
    }
}