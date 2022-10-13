using EventManager;
using System;
public class TestArsisIntEvent : IArsisEvent
{
    private int value;
    
    public override string eventId { get { return "thisIsMyTestID"; } }


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