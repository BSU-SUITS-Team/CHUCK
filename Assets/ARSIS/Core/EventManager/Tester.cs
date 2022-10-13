using UnityEngine;
using EventManagerSystem;
using UnityEngine.InputSystem;

public class Tester : MonoBehaviour {
    void Start() {
        EventManager.AddListener((TestArsisIntEvent e) => {
            Debug.Log("The value is: " + (int)e);
        });
    }

    void Update() {
        if (Keyboard.current.spaceKey.wasPressedThisFrame) {
            EventManager.Trigger(new TestArsisIntEvent(2));
        }
    }
}