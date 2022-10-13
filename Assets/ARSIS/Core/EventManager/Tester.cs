using UnityEngine;
using EventManager;
using UnityEngine.InputSystem;

public class Tester : MonoBehaviour {
    void Start() {
        EventManager.EventManager.AddListener((TestArsisIntEvent e) => {
            Debug.Log("Hello");
        });
    }

    void Update() {
        if (Keyboard.current.spaceKey.wasPressedThisFrame) {
            EventManager.EventManager.Trigger(new TestArsisIntEvent(2));
        }
    }
}