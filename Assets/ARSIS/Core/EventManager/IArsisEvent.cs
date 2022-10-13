using System.Diagnostics;
using System.Reflection;

namespace EventManagerSystem
{    
    /// <summary>
    /// This is an interface that is used to ensure a level of
    /// uniformity with what events can do.
    /// </summary>
    public abstract class IArsisEvent {
        public string whoCalledMe;
        public IArsisEvent() {
            //get the method that is calling this constructor
            MethodBase caller = new StackTrace().GetFrame(2).GetMethod();
            whoCalledMe = caller.DeclaringType.Name + "." + caller.Name;
        }
    }
}