using EventManagerSystem;
using System;
public class TestArsisIntEvent : IArsisEvent
{
    private int value;

    public TestArsisIntEvent() {
        value = 0;
    }
    
    public TestArsisIntEvent(int value) {
        this.value = value;
    }

    public override T ConvertFromEvent<T>()
    {
        return (T)Convert.ChangeType(value, typeof(T));
    }
}