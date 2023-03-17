using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Networking;

public class RoverController : MonoBehaviour
{

    public float lastActionTime = 0;
    public int postingRate = 5;
    public InputAction input;
    public Vector2 inputVector;
    public Vector2 lastInputVector;
    // Update is called once per frame
    void Update()
    {        
        if (Time.time - lastActionTime > 1f / postingRate) {
            lastActionTime = Time.time;
            if (inputVector != lastInputVector) {
                lastInputVector = inputVector;
                Debug.Log("Posting: " + format(inputVector));
                UnityWebRequest.Post("http://localhost:8181/devices/broadcast?command=" + format(inputVector), "").SendWebRequest();
            }
        }
        if (!input.IsInProgress()) {
            inputVector = Vector2.zero;
        }
    }

    private void Start() {        
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
        inputVector = context.ReadValue<Vector2>();
    }
}
