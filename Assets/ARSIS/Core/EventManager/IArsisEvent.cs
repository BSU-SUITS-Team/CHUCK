namespace EventManagerSystem
{    
    public abstract class IArsisEvent {
        public abstract T ConvertFromEvent<T>();
    }
}