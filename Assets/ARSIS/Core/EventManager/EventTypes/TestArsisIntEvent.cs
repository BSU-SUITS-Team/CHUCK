using EventManagerSystem;
using System;
public class TestArsisIntEvent : IArsisEvent
{
    private int value;
    
    public TestArsisIntEvent(int value) {
        this.value = value;
    }

    public static implicit operator int(TestArsisIntEvent e) {
        return e.value;
    }
}