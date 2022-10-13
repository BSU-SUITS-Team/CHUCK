namespace EventManager
{    
    public abstract class IArsisEvent {
        public abstract T ConvertFromEvent<T>();
    }
}