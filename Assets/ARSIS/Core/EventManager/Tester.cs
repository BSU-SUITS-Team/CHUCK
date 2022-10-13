using UnityEngine;
using EventManagerSystem;
using UnityEngine.InputSystem;

public class Tester : MonoBehaviour {
    void Start() {
        EventManager.AddListener((TestArsisIntEvent e) => {
            Debug.Log("The value is: " + (int)e);
        });
        EventManager.AddListener(TestOxygenUpdateHandler);
    }

    void Update() {
        if (Keyboard.current.spaceKey.wasPressedThisFrame) {
            EventManager.Trigger(new TestArsisIntEvent(2));
        }

        //every secondish randomly post a new oxygen level between 0 and 100
        if (Time.time % 1 < Time.deltaTime) {
            EventManager.Trigger(new OxygenLevel(Random.Range(0, 100)));
        }
    }

    private void thisIsAnotherTest(TestArsisIntEvent e) {
        Debug.Log("twice the value is: " + e * 2);
    }

    private void TestOxygenUpdateHandler(OxygenLevel e) {
        Debug.Log("Oxygen level is: " + e);
    }
}