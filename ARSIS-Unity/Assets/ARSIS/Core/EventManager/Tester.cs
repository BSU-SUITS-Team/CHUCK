using UnityEngine;
using EventSystem;
using UnityEngine.InputSystem;

public class Tester : MonoBehaviour
{

    bool isAdded;
    void Start()
    {
        //These are the two primary ways of adding event listeners
        EventManager.AddListener<OxygenLevel>(TestOxygenUpdateHandler);
        // EventManager.AddListener<OxygenLevel>(TestOtherEventHandler); //This will throw a compile time type error
        EventManager.AddListener((OxygenLevel e) =>
        {
            Debug.Log("(From lambda) Oxygen level is: " + e.value);
        });

        isAdded = true;
    }

    void Update()
    {
        //every secondish randomly post a new oxygen level between 0 and 100
        if (Time.time % 1 < Time.deltaTime)
        {
            EventManager.Trigger(new OxygenLevel(Random.Range(0f, 100f)));
        }

        //if the space bar is pressed, toggle the first listener
        if (Keyboard.current.spaceKey.wasPressedThisFrame)
        {
            if (isAdded)
            {
                EventManager.RemoveListener<OxygenLevel>(TestOxygenUpdateHandler);
            }
            else
            {
                EventManager.AddListener<OxygenLevel>(TestOxygenUpdateHandler);
            }
            isAdded = !isAdded;
        }

    }

    private void TestOxygenUpdateHandler(OxygenLevel e)
    {
        Debug.Log("(From private method) Oxygen level is: " + e.value);
        Debug.Log("This method was called from: " + e.whoCalledMe);
    }

    // This is to test compile time type checking
    class OtherEventType : BaseArsisEvent
    {
        public string message;
        public OtherEventType(string message)
        {
            this.message = message;
        }
    }
    private void TestOtherEventHandler(OtherEventType e)
    {
        Debug.Log("This will never be called");
    }
}
