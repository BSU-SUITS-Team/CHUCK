using UnityEngine;
using EventManager;
public class Tester : MonoBehaviour {
    void Start() {
        EventManager.EventManager.AddListener((IArsisEvent e) => {
            Debug.Log("Hello");
        });
    }
}