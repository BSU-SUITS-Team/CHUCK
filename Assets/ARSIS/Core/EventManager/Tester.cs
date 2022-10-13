using UnityEngine;
using EventManagerSystem;
using UnityEngine.InputSystem;

public class Tester : MonoBehaviour {

    bool isAdded;
    void Start() {
        //These are the two primary ways of adding event listeners
        EventManager.AddListener<OxygenLevel>(TestOxygenUpdateHandler);
        EventManager.AddListener((OxygenLevel e) => {
            Debug.Log("(From lambda) Oxygen level is: " + (float)e);
        });

        isAdded = true;
    }

    void Update() {
        //every secondish randomly post a new oxygen level between 0 and 100
        if (Time.time % 1 < Time.deltaTime) {
            EventManager.Trigger(new OxygenLevel(Random.Range(0f, 100f)));
        }

        //if the space bar is pressed, toggle the first listener
        if (Keyboard.current.spaceKey.wasPressedThisFrame) {
            if (isAdded) {
                EventManager.RemoveListener<OxygenLevel>(TestOxygenUpdateHandler);
            } else {
                EventManager.AddListener<OxygenLevel>(TestOxygenUpdateHandler);
            }
            isAdded = !isAdded;
        }

    }

    private void TestOxygenUpdateHandler(OxygenLevel e) {
        Debug.Log("(From private method) Oxygen level is: " + (float)e);
    }
}