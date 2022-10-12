namespace EventManager
{    
    public abstract class IArsisEvent {

        public abstract string eventId { get; }
        public abstract T ConvertFromEvent<T>();
    }
}