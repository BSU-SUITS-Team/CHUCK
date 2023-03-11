using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Networking;

public class RoverController : MonoBehaviour
{
    public InputAction input;
    // Update is called once per frame
    void Update()
    {
        input.performed += OnInput;
    }

    private void OnEnable() {
        input.Enable();
    }

    private void OnDisable() {
        input.Disable();
    }

    private string format(Vector2 inputVector) {
        return ((int)((inputVector.x + 1) * 127.5) + 1).ToString() + ":" + ((int)((inputVector.y + 1) * 127.5) + 1).ToString();
    }

    private void OnInput(InputAction.CallbackContext context) {
        Debug.Log(context.ReadValue<Vector2>());
        UnityWebRequest.Post("http://localhost:8181/devices/broadcast?command=" + format(context.ReadValue<Vector2>()), "").SendWebRequest();
    }
}
