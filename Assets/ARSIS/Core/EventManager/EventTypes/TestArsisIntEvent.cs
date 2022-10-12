using EventManager;
using System;
class TestArsisIntEvent : IArsisEvent
{
    private int value;
    
    public override string eventId { get { return "thisIsMyTestID"; } }

    public TestArsisIntEvent(int value) {
        this.value = value;
    }

    public override T ConvertFromEvent<T>()
    {
        return (T)Convert.ChangeType(value, typeof(T));
    }
}