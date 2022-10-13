using UnityEngine;
using EventManager;
public class Tester : MonoBehaviour {
    void Start() {
        EventManager.EventManager.AddListener((TestArsisIntEvent e) => {
            Debug.Log("Hello");
        });
    }
}